/****************************************************************************
*
*  Module Name    : PTP.c
*  Version        : 
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : van Kempen Bertrand
*  Date           : 27/07/2010
*  Modified by    : Baume Florian
*  Date           : 14/04/2016
*  Modification   : Linux driver port, removed floating point,
*                   changed time ref unit, stabiliy increased
*  Modified by    : multi-pcm-stage1 fork
*  Date           : 06/2026
*  Modification   : multi-rate W5 — per-rate TIC machinery extracted to
*                   tic_engine.c; this file is the PTP discipline (servo)
*                   only, fanning measurements out to attached engines
*  Known problems : None
*
* Copyright(C) 2017 Merging Technologies
*
* RAVENNA/AES67 ALSA LKM is free software; you can redistribute it and / or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* RAVENNA/AES67 ALSA LKM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with RAVAENNA ALSA LKM ; if not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#include "PTP.h"

#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include "c_wrapper_lib.h" //f10b use for module... to define into that file

#include "module_timer.h"

#include "EtherTubeInterfaces.h"

#include "MTAL_DP.h"

#define PTP_LOCK_HYSTERESIS		4
#define PTP_WATCHDOG_ELAPSE		20000	// we assume to receive at least one sync each 2s
/* TIC_LOCK_HYSTERESIS, PS_2_REF_UNIT and NS_2_REF_UNIT moved to
 * tic_engine.h (W5 per-rate TIC engine extraction). */

// PTP Domain
#define PTPMASTER_ANNOUNCE_TIMEOUT	50000000 // [100ns]


//////////////////////////////////////////////////////////////
void get_ptp_global_times(TClock_PTP* self, uint64_t* pui64GlobalSAC, uint64_t* pui64GlobalTime, uint64_t* pui64GlobalPerformanceCounter) // get the time and the SAC atomically
{
    tic_engine_get_times(&self->m_TicEngine, pui64GlobalSAC, pui64GlobalTime, pui64GlobalPerformanceCounter);
}

///////////////////////////////////////////////////////////////////////////////
// Helpers
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
static uint64_t GetSeconds(uint8_t * bySeconds)
{
    uint32_t dw;
    uint64_t ui64 = 0;
    for(dw = 0; dw < 6; dw++)
    {
        ui64 += (uint64_t)(bySeconds[dw]) << (40 - 8 * dw);
    }
    return ui64;
}


///////////////////////////////////////////////////////////////////////////////
#if 0
static void DumpPTPV2MsgHeader(TV2MsgHeader* pV2MsgHeader)
{
    int i;
	MTAL_DP("PTP v2 Msg Header\n");

	MTAL_DP("\tMessageLength: %d\n", MTAL_SWAP16(pV2MsgHeader->wMessageLength));
	MTAL_DP("\tClockIdentity: 0x");	for(i =0; i < 8; i++) MTAL_DP("%02x", pV2MsgHeader->SourcePortId.byClockIdentity[i]); MTAL_DP("\n");

	MTAL_DP("\tSourcePortId: %d\n", MTAL_SWAP16(pV2MsgHeader->SourcePortId.ui16PortNumber));


	MTAL_DP("\tSequence Id: %d\n", MTAL_SWAP16(pV2MsgHeader->wSequenceId));
	MTAL_DP("\tControl: %d\n", pV2MsgHeader->byControl);
}

///////////////////////////////////////////////////////////////////////////////
static void DumpV2TimeRepresentation(TV2TimeRepresentation* pOriginTimestamp)
{
	MTAL_DP("\tOriginTimestamp.Seconds: %llu\n", GetSeconds(pOriginTimestamp->bySeconds));
	MTAL_DP("\tOriginTimestamp.Nanoseconds: %d\n", (int32_t)MTAL_SWAP32(pOriginTimestamp->i32Nanoseconds));
}
#endif


//////////////////////////////////////////////////////////////
bool init_ptp(TClock_PTP* self, TEtherTubeNetfilter* pEth_netfilter, clock_ptp_ops* audio_streamer_clock_PTP_callback_ptr)
{
    self->m_pEth_netfilter = NULL;

	self->m_bInitialized = false;
	self->m_audio_streamer_clock_PTP_callback_ptr = audio_streamer_clock_PTP_callback_ptr;

	self->m_wLastSyncSequenceId = 0;
	self->m_wLastFollowUp = 0;

	self->m_ui64T2 = 0;
	self->m_ui64DeltaT2 = 0;

	self->m_usPTPLockCounter = PTP_LOCK_HYSTERESIS;

	////////////// new algorithm
	self->m_i64TIC_PTPToRTXClockOffset = 0;

	self->m_wLastDelayReqSequenceId = 0;
	memset(&self->m_PTPV2MsgDelayReqPacket, 0, sizeof(self->m_PTPV2MsgDelayReqPacket));


	self->m_ui64LastWatchDogTime = 0;
	self->m_wLastWatchDogSyncSequenceId = 0;

	// PTP info
	self->m_ui64PTP_GMID = 0;
	self->m_ui8PTPClockDomain = 0;

	/* Multi-rate W5: all rate-keyed TIC state lives in the engine. Step 1:
	 * one embedded engine, attached here — behaviorally identical to the
	 * pre-extraction single-rate servo. Step 2 moves engine ownership into
	 * the manager's (domain, rate) timer registry. */
	tic_engine_init(&self->m_TicEngine, self);
	self->m_apEngines[0] = &self->m_TicEngine;
	self->m_uNumEngines = 1;

    //////////////
	self->m_pEth_netfilter = pEth_netfilter;
	// PTP Master
	SetPTPMasterPortNumber(self, 1); // by default we are using Master 1

	self->m_bInitialized = true;

    self->m_csPTPTime = (void*)kmalloc(sizeof(spinlock_t), GFP_ATOMIC/*GFP_KERNEL*/);
    memset(self->m_csPTPTime, 0, sizeof(spinlock_t));
    spin_lock_init((spinlock_t*)self->m_csPTPTime);

    //######################################################
    memset(&self->m_PTPConfig, 0, sizeof(TPTPConfig));
    self->m_ui32PTPConfigChangedCounter = self->m_ui32LastPTPConfigChangedCounter = 0;
    ResetPTPMaster(self);
    //######################################################

	return true;
}

///////////////////////////////////////////////////////////////////////////////
void destroy_ptp(TClock_PTP* self)
{
	StopAudioFrameTICTimer(self);

	tic_engine_destroy(&self->m_TicEngine);

    kfree(self->m_csPTPTime);

	self->m_bInitialized = false;
}

///////////////////////////////////////////////////////////////////////////////
void ResetPTPLock(TClock_PTP* self, bool bUseMutex)
{
	if(bUseMutex)
		spin_lock((spinlock_t*)self->m_csPTPTime);

	{
		unsigned int e;
		MTAL_DP("[%u] ResetPTPLock()\n", self->m_pEth_netfilter->nic_id);
		self->m_usPTPLockCounter = PTP_LOCK_HYSTERESIS;

		/* W5: PTP-level events legitimately disturb every rate — reset all
		 * attached engines (IGR + TIC lock counter). */
		for (e = 0; e < self->m_uNumEngines; e++)
			tic_engine_reset(self->m_apEngines[e]);
	}

	if(bUseMutex)
		spin_unlock((spinlock_t*)self->m_csPTPTime);
}

///////////////////////////////////////////////////////////////////////////////
void SetPTPMasterPortNumber(TClock_PTP* self, unsigned short const usPTPMasterPortNumber)
{
	MTAL_DP("[%u] SetPTPMasterPortNumber %u\n", self->m_pEth_netfilter->nic_id, usPTPMasterPortNumber);
	self->m_usPTPMasterPortNumber = usPTPMasterPortNumber;

	// Enable PTP stamping for Master self->m_usPTPMasterPortNumber
	EnablePTPTimeStamping(self->m_pEth_netfilter, true, self->m_usPTPMasterPortNumber);
}

///////////////////////////////////////////////////////////////////////////////
/* W5: "is this servo driving audio" = any attached engine started. The
 * unlocked volatile read mirrors the old m_bAudioFrameTICTimerStarted gate. */
static bool clock_ptp_has_started_engine(TClock_PTP* self)
{
	unsigned int e;
	for (e = 0; e < self->m_uNumEngines; e++)
	{
		if (self->m_apEngines[e]->m_bAudioFrameTICTimerStarted)
			return true;
	}
	return false;
}
///////////////////////////////////////////////////////////////////////////////
EDispatchResult process_PTP_packet(TClock_PTP* self, TUDPPacketBase* pUDPPacketBase, uint32_t ui32PacketSize)
{
    TPTPPacketBase* pPTPPacketBase = (TPTPPacketBase*)pUDPPacketBase;
	if(!self->m_bInitialized || !clock_ptp_has_started_engine(self))
	{
		return DR_PACKET_NOT_USED;
	}

	if(pUDPPacketBase->UDPHeader.usDestPort != MTAL_SWAP16(319) && pUDPPacketBase->UDPHeader.usDestPort != MTAL_SWAP16(320))
	{ // 319: PTP Event; 320: PTP General
		return DR_PACKET_NOT_USED;
	}

	if(ui32PacketSize < sizeof(TPTPPacketBase))
	{
		MTAL_DP("[%u] too short PTP packet size = %u should be at least %u\n", self->m_pEth_netfilter->nic_id, ui32PacketSize, (uint32_t)sizeof(TPTPPacketBase));
		return DR_PACKET_NOT_USED;
	}

	// verify checksum
#if 0
	if (pUDPPacketBase->UDPHeader.usCheckSum != 0)
	{
		uint16_t ui16CheckSum = MTAL_ComputeUDPChecksum(&pPTPPacketBase->UDPHeader, MTAL_SWAP16(pUDPPacketBase->UDPHeader.usLen), (unsigned short*)&pPTPPacketBase->IPV4Header.ui32SrcIP, (unsigned short*)&pPTPPacketBase->IPV4Header.ui32DestIP);
		if (ui16CheckSum != 0)
		{
			MTAL_DP("[%u] Bad checksum 0x%x\n", self->m_pEth_netfilter->nic_id, ui16CheckSum);
			MTAL_DP("PTP type: 0%x seq_id: %u\n", pPTPPacketBase->V2MsgHeader.byTransportSpecificAndMessageType & 0x0F, pPTPPacketBase->V2MsgHeader.wSequenceId);
			MTAL_DP("\n");

			// the packet is rejected
			return DR_PACKET_ERROR;
		}
	}
#endif

	//DumpPTPV2MsgHeader(&pPTPPacketBase->V2MsgHeader);

	if((pPTPPacketBase->V2MsgHeader.byReserved1AndVersionPTP & 0xF) != 2)	// PTP version 2s
	{
		MTAL_DP("[%u] Incompatible PTP version\n", self->m_pEth_netfilter->nic_id);
		return DR_PACKET_NOT_USED;
	}

	switch(pPTPPacketBase->V2MsgHeader.byTransportSpecificAndMessageType & 0x0F)
	{
	case PTP_ANNOUNCE_MESSAGE:
        {
			TPTPV2MsgAnnouncePacket* pPTPV2MsgAnnouncePacket = (TPTPV2MsgAnnouncePacket*)pUDPPacketBase;
			//printk("PTP_ANNOUNCE_MESSAGE\n");
			if(ui32PacketSize < sizeof(TPTPV2MsgAnnouncePacket))
			{
				MTAL_DP("[%u] too short announce PTP packet size = %d should be at least %u\n", self->m_pEth_netfilter->nic_id, ui32PacketSize,  (uint32_t)sizeof(TPTPV2MsgAnnouncePacket));
				return DR_PACKET_NOT_USED;
			}

			{
				uint16_t wDeltaSeq = MTAL_SWAP16(pPTPV2MsgAnnouncePacket->V2MsgHeader.wSequenceId) - self->m_wLastAnnounceSequenceId;
				if (wDeltaSeq > 1)
				{
					MTAL_DP("[%u] PTP Announce delta(%u) error\n", self->m_pEth_netfilter->nic_id, wDeltaSeq);
					MTAL_DP("\tV2MsgHeader.wSequenceId = %d != m_wLastAnnounceSequenceId = %d + 1\n", MTAL_SWAP16(pPTPV2MsgAnnouncePacket->V2MsgHeader.wSequenceId), self->m_wLastAnnounceSequenceId);
				}
				self->m_wLastAnnounceSequenceId = MTAL_SWAP16(pPTPV2MsgAnnouncePacket->V2MsgHeader.wSequenceId);
			}

            //######################################################
            {
                uint64_t ui64_CurrentClockIdentity = *(uint64_t*)pPTPV2MsgAnnouncePacket->V2MsgHeader.SourcePortId.byClockIdentity;
                
                // is PTPConfig has changed?
                if (self->m_ui32PTPConfigChangedCounter != self->m_ui32LastPTPConfigChangedCounter)
                { // restart election
                    self->m_ui32LastPTPConfigChangedCounter = self->m_ui32PTPConfigChangedCounter;
                    
                    MTAL_DP("[%u] PTPConfig has changed -> restart election\n", self->m_pEth_netfilter->nic_id);
                    
                    ResetPTPLock(self, true);
                    ResetPTPMaster(self);
                }
                
                // Is elected PTPMaster's domain still match wanted domain?
                if (ui64_CurrentClockIdentity == self->m_ui64PTPMaster_ClockIdentity
                    && pPTPV2MsgAnnouncePacket->V2MsgHeader.byDomainNumber != self->m_PTPConfig.ui8Domain)
                { // restart election
                     MTAL_DP("[%u] PTP Master domain no longer matches wanted domain -> restart election\n", self->m_pEth_netfilter->nic_id);
                    ResetPTPLock(self, true);
                    ResetPTPMaster(self);
                }
                
                if (pPTPV2MsgAnnouncePacket->V2MsgHeader.byDomainNumber == self->m_PTPConfig.ui8Domain)
                {
                    bool bElectThisPTPMaster = false;
                    uint64_t ui64CurrentTime;
                    get_clock_time(&ui64CurrentTime);
                    ui64CurrentTime /= 100; // [100ns]
                    
                    // very simple PTPMaster election
                    // 1. no PTPMaster elected then use the first one
                    // 2. if no long receive announce from this PTPMaster then use the first one
                    
                    if (self->m_ui64PTPMaster_ClockIdentity == 0)
                    { // never receive any announce or election restarted from scratch
                        bElectThisPTPMaster = true;
                    }
                    if (self->m_ui64PTPMaster_AnnounceTime != 0 && ui64CurrentTime - self->m_ui64PTPMaster_AnnounceTime > PTPMASTER_ANNOUNCE_TIMEOUT)
                    { // too long time we didn't receive the announce -> restart election
                        MTAL_DP("[%u] PTP Master announce timeout\n", self->m_pEth_netfilter->nic_id);
                        bElectThisPTPMaster = true;
                    }
                    
                    if(bElectThisPTPMaster)
                    { // use this PTP Master
                        printk("[%u] Use this PTP Master\n", self->m_pEth_netfilter->nic_id);
                        ResetPTPLock(self, true);
                        ResetPTPMaster(self);
                        
                        self->m_ui64PTPMaster_ClockIdentity = ui64_CurrentClockIdentity;
                    }
                    
                    // Is elected PTPMaster?
                    if (ui64_CurrentClockIdentity == self->m_ui64PTPMaster_ClockIdentity)
                    {   
                        // update GMID and save announce info
                        memcpy(&self->m_PTPMaster_Announce, &pPTPV2MsgAnnouncePacket->V2MsgAnnounce, sizeof(TV2MsgAnnounce));
                        if (*(uint64_t*)pPTPV2MsgAnnouncePacket->V2MsgAnnounce.byGrandmasterClockIdentity != self->m_ui64PTPMaster_GMID)
                        {
                            printk("[%u] Updating PTP Master GMID to %llu\n", self->m_pEth_netfilter->nic_id, *(uint64_t*)pPTPV2MsgAnnouncePacket->V2MsgAnnounce.byGrandmasterClockIdentity);
                        }
                        // save announce time
                        self->m_ui64PTPMaster_GMID = *(uint64_t*)pPTPV2MsgAnnouncePacket->V2MsgAnnounce.byGrandmasterClockIdentity;
                        self->m_ui64PTPMaster_AnnounceTime = ui64CurrentTime;
                        MTAL_DP("[%u] Updating announce time %llu\n", self->m_pEth_netfilter->nic_id, self->m_ui64PTPMaster_AnnounceTime);
                    }
                }
				else
				{
					MTAL_DP("[%u] Announced domain %d look for domain %d\n", self->m_pEth_netfilter->nic_id, pPTPV2MsgAnnouncePacket->V2MsgHeader.byDomainNumber, self->m_PTPConfig.ui8Domain);
				}

            }
            //######################################################
            
			break;
		}
		case PTP_SYNC_MESSAGE:
		{
            uint64_t ui64T2;
            uint16_t wDeltaSeq;
			/*MTAL_RtTraceEvent(RTTRACEEVENT_PTP_SYNC, (PVOID)(RT_TRACE_EVENT_SIGNAL_STOP), 0);
			MTAL_RtTraceEvent(RTTRACEEVENT_PTP_SYNC, (PVOID)(RT_TRACE_EVENT_SIGNAL_START), 0);*/

			TPTPV2MsgSyncPacket* pPTPV2MsgSyncPacket = (TPTPV2MsgSyncPacket*)pUDPPacketBase;
			//printk("PTP_SYNC_MESSAGE\n");
			if(ui32PacketSize < sizeof(TPTPV2MsgSyncPacket))
			{
				MTAL_DP("[%u] too short PTP packet size = %d should be at least %u\n", self->m_pEth_netfilter->nic_id, ui32PacketSize, (uint32_t)sizeof(TPTPV2MsgSyncPacket));
				return DR_PACKET_NOT_USED;
			}
			//DumpV2TimeRepresentation(&pPTPV2MsgSyncPacket->V2MsgSync.OriginTimestamp);
            
            //######################################################
            // Check PTP Clock identity
            if (*(uint64_t*)pPTPV2MsgSyncPacket->V2MsgHeader.SourcePortId.byClockIdentity != self->m_ui64PTPMaster_ClockIdentity)
            { // ignore this packet
                //MTAL_DP("PTP sync packet filtered wrong ClockIdentity %I64X expected %I64X\n", *(uint64_t*)pPTPV2MsgSyncPacket->V2MsgHeader.SourcePortId.byClockIdentity, self->m_ui64PTPMaster_ClockIdentity);
                return DR_RTP_PACKET_USED;
            }
            //######################################################

			get_clock_time(&ui64T2); // retrieve the packet arrival time (RTX clock domain).
			ui64T2 /= NS_2_REF_UNIT; // [100ns]
			/*if(!GetPTPTimeStamp(self->m_pEth_netfilter, &ui64T2)) // [100ns]	// retrieve the packet arrival time (RTX clock domain).
			{
				MTAL_DP("[%u] GetPTPTimeStamp failed\n", self->m_pEth_netfilter->nic_id);
				return DR_PACKET_NOT_USED;
			}*/

			wDeltaSeq = MTAL_SWAP16(pPTPV2MsgSyncPacket->V2MsgHeader.wSequenceId) - self->m_wLastSyncSequenceId;
			if(wDeltaSeq > PTP_LOCK_HYSTERESIS)	// we check that sync packets are contiguous
			{
				self->m_ui64DeltaT2 = 0;

				ResetPTPLock(self, true);
				MTAL_DP("[%u] PTP Sync delta(%u) error: -> reset PTP internal locked\n", self->m_pEth_netfilter->nic_id, wDeltaSeq);
				MTAL_DP("\tV2MsgHeader.wSequenceId = %d != self->m_wLastSyncSequenceId = %d + 1\n", MTAL_SWAP16(pPTPV2MsgSyncPacket->V2MsgHeader.wSequenceId), self->m_wLastSyncSequenceId);
			}
			else
			{
				// Atomicity
                {
                    spin_lock((spinlock_t*)self->m_csPTPTime);

					self->m_ui64DeltaT2 = ui64T2 - self->m_ui64T2;
					//MTAL_DP("%I64u Delta T2= %I64u\n", ui64T2, ui64T2 - self->m_ui64T2);

					/*MTAL_RtTraceEvent(RTTRACEEVENT_PTP_SYNC, (PVOID)(RT_TRACE_EVENT_SIGNAL_STOP), 0);
					MTAL_RtTraceEvent(RTTRACEEVENT_PTP_SYNC, (PVOID)(RT_TRACE_EVENT_SIGNAL_START), (PVOID)(unsigned int)self->m_ui64DeltaT2);*/

					//////////////////////////////////////////////
					// Syntonization
					{
						unsigned int e;
						self->m_ui64T2 = ui64T2;
						/* W5: snapshot each engine's last tick time at T2 */
						for (e = 0; e < self->m_uNumEngines; e++)
							tic_engine_snapshot_at_t2(self->m_apEngines[e]);
					}

                    spin_unlock((spinlock_t*)self->m_csPTPTime);
				}

				//MTAL_DP("Flags: 0x%x\n", MTAL_SWAP16(pPTPV2MsgSyncPacket->V2MsgHeader.wFlags));
				if (!IS_PTP_TWO_STEP(pPTPV2MsgSyncPacket->V2MsgHeader.wFlags))
				{	// no follow_up; so we use the time from the Sync packet
					uint64_t ui64T1 = GetSeconds(pPTPV2MsgSyncPacket->V2MsgSync.OriginTimestamp.bySeconds) * 1000000000 + (int32_t)MTAL_SWAP32(pPTPV2MsgSyncPacket->V2MsgSync.OriginTimestamp.i32Nanoseconds); // [ns]
																																																			   // Correction field
					int64_t i64Correction = MTAL_SWAP64(pPTPV2MsgSyncPacket->V2MsgHeader.i64CorrectionField) >> 16;
					ui64T1 += i64Correction;
					ui64T1 /= NS_2_REF_UNIT; // [100ns]

					ProcessT1(self, ui64T1);
				}
			}

			self->m_wLastSyncSequenceId = MTAL_SWAP16(pPTPV2MsgSyncPacket->V2MsgHeader.wSequenceId);

			break;
		}

		case PTP_FOLLOWUP_MESSAGE:
		{
			TPTPV2MsgFollowUpPacket* pPTPV2MsgFollowUpPacket = (TPTPV2MsgFollowUpPacket*)pUDPPacketBase;
			if(ui32PacketSize < sizeof(TPTPV2MsgFollowUpPacket))
			{
				MTAL_DP("[%u] too short PTP packet size = %d should be at least %u\n", self->m_pEth_netfilter->nic_id, ui32PacketSize, (uint32_t)sizeof(TPTPV2MsgFollowUpPacket));
				return DR_PACKET_NOT_USED;
			}
			//printk("PTP_FOLLOWUP_MESSAGE\n");
			//DumpV2TimeRepresentation(&pPTPV2MsgFollowUpPacket->V2MsgFollowUp.PreciseOriginTimestamp);
            
            //######################################################
            // Check PTP Clock identity
            if (*(uint64_t*)pPTPV2MsgFollowUpPacket->V2MsgHeader.SourcePortId.byClockIdentity != self->m_ui64PTPMaster_ClockIdentity)
            { // ignore this packet
                //MTAL_DP("PTP follow_up packet filtered wrong ClockIdentity %I64X expected %I64X\n", *(uint64_t*)pPTPV2MsgFollowUpPacket->V2MsgHeader.SourcePortId.byClockIdentity, self->m_ui64PTPMaster_ClockIdentity);
                return DR_RTP_PACKET_USED;
            }
            //######################################################
            {
                uint64_t ui64T1 = GetSeconds(pPTPV2MsgFollowUpPacket->V2MsgFollowUp.PreciseOriginTimestamp.bySeconds) * 1000000000 + (int32_t)MTAL_SWAP32(pPTPV2MsgFollowUpPacket->V2MsgFollowUp.PreciseOriginTimestamp.i32Nanoseconds); // [ns]
                // Correction field
                int64_t i64Correction = MTAL_SWAP64(pPTPV2MsgFollowUpPacket->V2MsgHeader.i64CorrectionField) >> 16;
                ui64T1 += i64Correction;
                ui64T1 /= NS_2_REF_UNIT; // [100ns]


                if(MTAL_SWAP16(pPTPV2MsgFollowUpPacket->V2MsgHeader.wSequenceId) == self->m_wLastSyncSequenceId) // we verify that this follow up match the last sync packet received.
                    //&& MTAL_SWAP16(pPTPV2MsgFollowUpPacket->V2MsgHeader.wSequenceId) == self->m_wLastFollowUp + 1) // we don't care if a follow up is missing
                {
                    ProcessT1(self, ui64T1);
                }
                else
                {
                    MTAL_DP("This FollowUp(seq = %d) doesn't match the last sync(seq = %d) received\n",  MTAL_SWAP16(pPTPV2MsgFollowUpPacket->V2MsgHeader.wSequenceId), self->m_wLastSyncSequenceId);
                }
                self->m_wLastFollowUp = MTAL_SWAP16(pPTPV2MsgFollowUpPacket->V2MsgHeader.wSequenceId);

                //SendDelayReq(self, pPTPV2MsgFollowUpPacket);
            }
			break;
		}
		case PTP_DELAY_REQ_MESSAGE:
		case PTP_PATH_DELAY_REQ_MESSAGE:
		case PTP_PATH_DELAY_RESP_MESSAGE:
		case PTP_DELAY_RESP_MESSAGE:
		case PTP_PATH_DELAY_FOLLOWUP_MESSAGE:
		case PTP_SIGNALLING_MESSAGE:
		case PTP_MANAGEMENT_MESSAGE:
            break;
		default:
		{
			MTAL_DP("[%u] Unknown PTPv2 message type\n", self->m_pEth_netfilter->nic_id);
			break;
		}
	}
	return DR_PTP_PACKET_USED;
}

//######################################################
void ResetPTPMaster(TClock_PTP* self)
{
    printk("[%u] ResetPTPMaster\n", self->m_pEth_netfilter->nic_id);
    memset(&self->m_PTPMaster_Announce, 0, sizeof(TV2MsgAnnounce));
    self->m_ui64PTPMaster_AnnounceTime = 0;
    self->m_ui64PTPMaster_ClockIdentity = 0;
    self->m_ui64PTPMaster_GMID = 0;
}
//######################################################

///////////////////////////////////////////////////////////////////////////////
// from Sync or Follow_up
void ProcessT1(TClock_PTP* self, uint64_t ui64T1)
{
	uint64_t ui64DeltaT1 = ui64T1 - self->m_ui64T1;
	if (ui64DeltaT1 == 0)
	{
		MTAL_DP("[%u] ui64DeltaT1 = %llu, current TIC period not proceed in order to prevent a 0 division !!\n", self->m_pEth_netfilter->nic_id, ui64DeltaT1);
		return;
	}
	// Atomicity
	{
		unsigned int e;
		spin_lock((spinlock_t*)self->m_csPTPTime);
		if (self->m_usPTPLockCounter > 0)
		{
			self->m_usPTPLockCounter--;
			if (self->m_usPTPLockCounter == 0)
			{
				MTAL_DP("[%u] PTP locked\n", self->m_pEth_netfilter->nic_id);
			}
			else
			{
				MTAL_DP("[%u] PTP lock pending (%d)\n", self->m_pEth_netfilter->nic_id, self->m_usPTPLockCounter);
				if (self->m_usPTPLockCounter == 1)
				{
					/* W5: initial period syntonization + tick phase placement
					 * is rate-keyed — fan out to every attached engine. */
					for (e = 0; e < self->m_uNumEngines; e++)
						tic_engine_prelock_phase_init(self->m_apEngines[e], ui64T1, ui64DeltaT1);
				}
			}
		}

		if (self->m_usPTPLockCounter == 0)
		{
			self->m_i64TIC_PTPToRTXClockOffset = (int64_t)self->m_ui64T2 - (int64_t)ui64T1;

			/* W5: frame alignment + PI steering is rate-keyed — fan out. */
			for (e = 0; e < self->m_uNumEngines; e++)
				tic_engine_steer(self->m_apEngines[e], ui64T1);
		}
		spin_unlock((spinlock_t*)self->m_csPTPTime);
	}

	self->m_ui64T1 = ui64T1;
}

////////////////////////////////////////////////////////////////////
bool SendDelayReq(TClock_PTP* self, TPTPV2MsgFollowUpPacket* pPTPV2MsgFollowUpPacket)
{
	memcpy(&self->m_PTPV2MsgDelayReqPacket, pPTPV2MsgFollowUpPacket, sizeof(TPTPV2MsgFollowUpPacket));

	// Ethernet
	memcpy(self->m_PTPV2MsgDelayReqPacket.EthernetHeader.byDest, self->m_PTPV2MsgDelayReqPacket.EthernetHeader.bySrc, 6);
	GetMACAddress(self->m_pEth_netfilter, self->m_PTPV2MsgDelayReqPacket.EthernetHeader.bySrc, 6);
#ifdef WIRESHARK_DEBUG
	self->m_PTPV2MsgDelayReqPacket.EthernetHeader.byDest[5] = 0xFF;
#endif //WIRESHARK_DEBUG

	//IP
	self->m_PTPV2MsgDelayReqPacket.IPV4Header.ui32DestIP = self->m_PTPV2MsgDelayReqPacket.IPV4Header.ui32SrcIP;
	self->m_PTPV2MsgDelayReqPacket.IPV4Header.ui32SrcIP = MTAL_SWAP32(self->m_audio_streamer_clock_PTP_callback_ptr->GetIPAddress(self->m_audio_streamer_clock_PTP_callback_ptr->user));
	self->m_PTPV2MsgDelayReqPacket.IPV4Header.byTTL = 128;
	self->m_PTPV2MsgDelayReqPacket.IPV4Header.usChecksum = 0;
	self->m_PTPV2MsgDelayReqPacket.IPV4Header.usChecksum = MTAL_SWAP16(MTAL_ComputeChecksum(&self->m_PTPV2MsgDelayReqPacket.IPV4Header, sizeof(TIPV4Header)));

	// UDP
	self->m_PTPV2MsgDelayReqPacket.UDPHeader.usSrcPort = MTAL_SWAP16(319);
	self->m_PTPV2MsgDelayReqPacket.UDPHeader.usDestPort = MTAL_SWAP16(319);
	self->m_PTPV2MsgDelayReqPacket.UDPHeader.usCheckSum = 0;

	// PTP
	self->m_PTPV2MsgDelayReqPacket.V2MsgHeader.byTransportSpecificAndMessageType &= 0xF0;                   // Clear previous Message type
	self->m_PTPV2MsgDelayReqPacket.V2MsgHeader.byTransportSpecificAndMessageType |= PTP_DELAY_REQ_MESSAGE;

	self->m_PTPV2MsgDelayReqPacket.V2MsgHeader.wMessageLength = MTAL_SWAP16(sizeof(TV2MsgHeader) + sizeof(TV2MsgSync));

	self->m_PTPV2MsgDelayReqPacket.V2MsgHeader.wFlags = 0;

  /*
	if (unicast)
	 {
		*(UInteger8*) (buf + 6) |= V2_UNICAST_FLAG;
	}
	  */
	self->m_PTPV2MsgDelayReqPacket.V2MsgHeader.i64CorrectionField = 0;
	self->m_PTPV2MsgDelayReqPacket.V2MsgHeader.wSequenceId = ++self->m_wLastDelayReqSequenceId;
	self->m_PTPV2MsgDelayReqPacket.V2MsgHeader.byControl = PTP_DELAY_REQ_CONTROL;
	self->m_PTPV2MsgDelayReqPacket.V2MsgHeader.byLogMeanMessageInterval = 0;


	// Timestamp will be update by the NIC Driver; in fact (CEtherTubeNIC_RTXDriver)
	/*
	  *(UInteger16*)  (buf + 34) =  flip16(originTimestamp->epoch_number);
	  *(UInteger32*)  (buf + 36) =  flip32(originTimestamp->seconds);
	  *(UInteger32*)  (buf + 40) =  flip32(originTimestamp->nanoseconds);
	*/

	self->m_PTPV2MsgDelayReqPacket.UDPHeader.usCheckSum = MTAL_SWAP16(MTAL_ComputeUDPChecksum(&self->m_PTPV2MsgDelayReqPacket.UDPHeader, sizeof(TPTPV2MsgFollowUpPacket) - sizeof(TEthernetHeader)  - sizeof(TIPV4Header), (uint16_t*)&self->m_PTPV2MsgDelayReqPacket.IPV4Header.ui32SrcIP, (uint16_t*)&self->m_PTPV2MsgDelayReqPacket.IPV4Header.ui32DestIP));

	SendRawPacket(self->m_pEth_netfilter, &self->m_PTPV2MsgDelayReqPacket, sizeof(self->m_PTPV2MsgDelayReqPacket));

	return true;
}

///////////////////////////////////////////////////////////////////////////////
/*CMTAL_CriticalSectionBase* get_SAC_time_lock(TClock_PTP* self)
{
    return &self->m_csSAC_Time_Lock;
}*/


///////////////////////////////////////////////////////////////////////////////
/* computeNextAbsoluteTime moved to tic_engine.c (W5 extraction). */

///////////////////////////////////////////////////////////////////////////////
// Timer
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void timerSetNextAbsoluteTime(TClock_PTP* self, uint64_t ui64NextAbsoluteTime)
{
	tic_engine_set_next_abs_time(&self->m_TicEngine, ui64NextAbsoluteTime);
}

void timerProcess(TClock_PTP* self, uint64_t* pui64NextRTXClockTime, uint64_t ui64RTXClockTime)
{
	TTicEngineTickCtx ctx;

	/* W5 step 1: single embedded engine. The advance/schedule split (with
	 * the servo-level link/watchdog checks between them, exactly where the
	 * old monolithic timerProcess ran them) is the shape the per-(domain,
	 * rate) timer callbacks build on in step 2. */
	tic_engine_tick_advance(&self->m_TicEngine, &ctx);

	if (ctx.bStarted)
	{
		// Check the link status
		if(!IsLinkUp(self->m_pEth_netfilter) && GetLockStatus(self) != PTPLS_UNLOCKED)
		{
			spin_lock((spinlock_t*)self->m_csPTPTime);
			{
				MTAL_DP("[%u] PTP detects that the link is down\n", self->m_pEth_netfilter->nic_id);
				ResetPTPLock(self, false);
			}
			spin_unlock((spinlock_t*)self->m_csPTPTime);
		}

		// PTP watch dog
		{
			uint64_t ui64WatchDogElapse = ctx.ui64CurrentRTXClockTime - self->m_ui64LastWatchDogTime;
			if(ui64WatchDogElapse >= PTP_WATCHDOG_ELAPSE)
			{
				spin_lock((spinlock_t*)self->m_csPTPTime);
				if(self->m_wLastWatchDogSyncSequenceId == self->m_wLastSyncSequenceId && GetLockStatus(self) != PTPLS_UNLOCKED)
				{
					printk("[%u] PTP Master sync timeout, resetting ...\n", self->m_pEth_netfilter->nic_id);
					MTAL_DP("[%u] Didn't received PTP sync since 2s\n", self->m_pEth_netfilter->nic_id);
					MTAL_DP("[%u] ui64WatchDogElapse = %llu = %llu - %llu\n", self->m_pEth_netfilter->nic_id, ui64WatchDogElapse, ctx.ui64CurrentRTXClockTime, self->m_ui64LastWatchDogTime);
					ResetPTPLock(self, false);
				}
				spin_unlock((spinlock_t*)self->m_csPTPTime);
				self->m_wLastWatchDogSyncSequenceId = self->m_wLastSyncSequenceId;
				self->m_ui64LastWatchDogTime = ctx.ui64CurrentRTXClockTime;
			}
		}
	}

	tic_engine_tick_schedule(&self->m_TicEngine, &ctx, pui64NextRTXClockTime);
}

///////////////////////////////////////////////////////////////////////////////
bool StartAudioFrameTICTimer(TClock_PTP* self, uint32_t ulFrameSize, uint32_t ulSamplingRate)
{
	StopAudioFrameTICTimer(self);

	// Atomicity
	{
        spin_lock((spinlock_t*)self->m_csPTPTime);
		tic_engine_start(&self->m_TicEngine, ulFrameSize, ulSamplingRate);
		set_base_period(self->m_TicEngine.m_dTIC_BasePeriod/1000);
		spin_unlock((spinlock_t*)self->m_csPTPTime);
	}
	MTAL_DP("[%u] StartAudioFrameTICTimer with...\n", self->m_pEth_netfilter->nic_id);
	MTAL_DP("self->m_dTIC_BasePeriod = %llu	[ps]\n", self->m_TicEngine.m_dTIC_BasePeriod);
	MTAL_DP("self->m_ui32FrameSize = %u self->m_ui32SamplingRate = %u\n", self->m_TicEngine.m_ui32FrameSize, self->m_TicEngine.m_ui32SamplingRate);

	// samplingrate and/or framesize changed so computation made during PTP locking is no longer valid
	ResetPTPLock(self, true);

	return true;
}

///////////////////////////////////////////////////////////////////////////////
bool StopAudioFrameTICTimer(TClock_PTP* self)
{
	unsigned int e;
	for (e = 0; e < self->m_uNumEngines; e++)
		tic_engine_stop(self->m_apEngines[e]);

    ResetPTPLock(self, true);

	return true;
}

////////////////////////////////////////////////////////////////////
bool IsAudioFrameTICDropped(TClock_PTP* self, bool bReset)
{
	return tic_engine_is_drop(&self->m_TicEngine, bReset);
}

///////////////////////////////////////////////////////////////////////////////
EPTPLockStatus GetLockStatus(TClock_PTP* self)
{
	return tic_engine_lock_status(&self->m_TicEngine);
}

///////////////////////////////////////////////////////////////////////////////
void SetPTPConfig(TClock_PTP* self, TPTPConfig* pPTPConfig)
{
    if (!pPTPConfig)
    {
        return;
    }
    if (self->m_PTPConfig.ui8Domain != pPTPConfig->ui8Domain
        || self->m_PTPConfig.ui8DSCP != pPTPConfig->ui8DSCP)
    {
        self->m_PTPConfig = *pPTPConfig;
        self->m_ui32PTPConfigChangedCounter++;
        
        MTAL_DP("[%u] PTPConfig: domain = %u, DSCP = %u\n", self->m_pEth_netfilter->nic_id, self->m_PTPConfig.ui8Domain, self->m_PTPConfig.ui8DSCP);
    }
}

///////////////////////////////////////////////////////////////////////////////
void GetPTPConfig(TClock_PTP* self, TPTPConfig* pPTPConfig)
{
    if (!pPTPConfig)
    {
        return;
    }
    *pPTPConfig = self->m_PTPConfig;
}

///////////////////////////////////////////////////////////////////////////////
void GetPTPStatus(TClock_PTP* self, TPTPStatus* pPTPStatus)
{
    if (!pPTPStatus)
    {
        return;
    }
    memset(pPTPStatus, 0, sizeof(TPTPStatus));

    pPTPStatus->nPTPLockStatus = GetLockStatus(self);
    pPTPStatus->ui64GMID[0] = self->m_ui64PTPMaster_GMID;
    pPTPStatus->i32NetworkJitter = 0; // TODO
	pPTPStatus->i32ClockJitter = self->m_TicEngine.m_maxClkJitter;

	//MTAL_DP("[%u] CLK jitter = %u\n", self->m_pEth_netfilter->nic_id, self->m_maxClkJitter);
	self->m_TicEngine.m_maxClkJitter = 0;
}

///////////////////////////////////////////////////////////////////////////////
uint8_t GetPTPPriority(TClock_PTP* self)
{
	return self->m_PTPMaster_Announce.Priority1;
}

///////////////////////////////////////////////////////////////////////////////
/*void GetPTPStats(TClock_PTP* self, TPTPStats* pPTPStats)
{
	if(!pPTPStats)
	{
		return;
	}

	memset(pPTPStats, 0, sizeof(TPTPStats));
	{
        unsigned long flags;
        spin_lock((spinlock_t*)self->m_csPTPTime);

		pPTPStats->fPTPSyncRatio = self->m_pmmmPTPStatRatio.GetMax();
		self->m_pmmmPTPStatRatio.ResetAtNextPoint();

		pPTPStats->ui32PTPSyncMinArrivalDelta = self->m_pmmmPTPStatSyncInterval.GetMin() / 10; // [us]
		pPTPStats->ui32PTPSyncAvgArrivalDelta = self->m_pmmmPTPStatSyncInterval.GetAvg() / 10; // [us]
		pPTPStats->ui32PTPSyncMaxArrivalDelta = self->m_pmmmPTPStatSyncInterval.GetMax() / 10; // [us]
		self->m_pmmmPTPStatSyncInterval.ResetAtNextPoint();

		pPTPStats->ui32PTPFollowMinArrivalDelta	= self->m_pmmmPTPStatFollowInterval.GetMin() / 10; // [us]
		pPTPStats->ui32PTPFollowAvgArrivalDelta	= self->m_pmmmPTPStatFollowInterval.GetAvg() / 10; // [us]
		pPTPStats->ui32PTPFollowMaxArrivalDelta	= self->m_pmmmPTPStatFollowInterval.GetMax() / 10; // [us]
		self->m_pmmmPTPStatFollowInterval.ResetAtNextPoint();



		pPTPStats->i32PTPMinDeltaTICFrame = self->m_pmmmPTPStatDeltaTICFrame.GetMin() / 10; // [us]
		pPTPStats->i32PTPAvgDeltaTICFrame = self->m_pmmmPTPStatDeltaTICFrame.GetAvg() / 10; // [us]
		pPTPStats->i32PTPMaxDeltaTICFrame = self->m_pmmmPTPStatDeltaTICFrame.GetMax() / 10; // [us]
		self->m_pmmmPTPStatDeltaTICFrame.ResetAtNextPoint();

		spin_unlock((spinlock_t*)self->m_csPTPTime);
	}
}

///////////////////////////////////////////////////////////////////////////////
void GetTICStats(TClock_PTP* self, TTICStats* pTICStats)
{
	if(!pTICStats)
	{
		return;
	}

	memset(pTICStats, 0, sizeof(TTICStats));
	{
        unsigned long flags;
        spin_lock((spinlock_t*)self->m_csPTPTime);

		pTICStats->ui32TICMinDelta = (uint32_t)self->m_pmiTICInterval.GetMin(); // us
		pTICStats->ui32TICMaxDelta = (uint32_t)self->m_pmiTICInterval.GetMax(); // [us]
		self->m_pmiTICInterval.ResetAtNextPoint();

		spin_unlock((spinlock_t*)self->m_csPTPTime);
	}
}*/

///////////////////////////////////////////////////////////////////////////////
uint64_t get_ptp_global_SAC(TClock_PTP* self)
{
    return tic_engine_get_sac(&self->m_TicEngine);
}

///////////////////////////////////////////////////////////////////////////////
uint64_t get_ptp_global_time(TClock_PTP* self)
{
    return tic_engine_get_time(&self->m_TicEngine);
}

///////////////////////////////////////////////////////////////////////////////
/* Multi-rate Stage 3: the sampling rate this clock's SAC is counted at.
 * W5: delegates to the servo's single step-1 engine. */
uint32_t get_ptp_sampling_rate(TClock_PTP* self)
{
    return tic_engine_get_rate(&self->m_TicEngine);
}
