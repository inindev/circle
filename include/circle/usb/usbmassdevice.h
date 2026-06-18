//
// usbmassdevice.h
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2014-2022  R. Stange <rsta2@o2online.de>
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _circle_usb_usbmassdevice_h
#define _circle_usb_usbmassdevice_h

#include <circle/usb/usbfunction.h>
#include <circle/usb/usbendpoint.h>
#include <circle/usb/usbrequest.h>
#include <circle/fs/partitionmanager.h>
#include <circle/numberpool.h>
#include <circle/types.h>

#define UMSD_BLOCK_SIZE		512
#define UMSD_BLOCK_MASK		(UMSD_BLOCK_SIZE-1)
#define UMSD_BLOCK_SHIFT	9

#define UMSD_MAX_OFFSET		0x1FFFFFFFFFFULL		// 2TB

// Command()/ReadCDDA() error result classes (mirrors Linux usb_stor transport
// status). A non-negative result is the byte count; both error codes are < 0, so
// callers that only test the sign keep treating either as a plain failure. A
// caller that wants Linux-style escalation can distinguish them:
//   UMSD_CMD_FAILED  ordinary/recoverable failure (stall mishandled, CSW bad,
//                    command rejected) -- a retry / soft reset is appropriate.
//   UMSD_CMD_ERROR   USB transaction error/timeout, or a desynced CSW -- the
//                    device is not responding on the bus. Per Linux
//                    (USB_STOR_TRANSPORT_ERROR), clearing halts / soft reset will
//                    NOT restore it; the owner must escalate to a port reset
//                    (re-enumerate the device).
#define UMSD_CMD_FAILED		(-1)
#define UMSD_CMD_ERROR		(-2)

class CUSBBulkOnlyMassStorageDevice : public CUSBFunction
{
public:
	CUSBBulkOnlyMassStorageDevice (CUSBFunction *pFunction);
	~CUSBBulkOnlyMassStorageDevice (void);

	boolean Configure (void);

	int Read (void *pBuffer, size_t nCount);
	int Write (const void *pBuffer, size_t nCount);

	u64 Seek (u64 ullOffset);

	u64 GetSize (void) const;		// in bytes
	unsigned GetCapacity (void) const;	// in blocks

	// Optical (CD/DVD) support. A USB CD drive enumerates as the same
	// Mass-Storage / SCSI-transparent / Bulk-Only class as a disk, so it binds
	// here too; it differs by SCSI peripheral device type (0x05) and 2048-byte
	// blocks. IsCDROM() lets callers branch; the block size is per-instance.
	boolean IsCDROM (void) const { return m_bCDROM; }
	unsigned GetBlockSize (void) const { return m_nBlockSize; }

	// Re-probe optical media: TEST UNIT READY, and (if ready) refresh capacity.
	// Returns TRUE if media is present and ready. Call when a disc may have been
	// inserted after Configure() (which runs once, possibly with no disc in).
	boolean RefreshMedia (void);

	// MMC READ TOC/PMA/ATIP (0x43), format 0: fills pBuffer with the raw TOC
	// response (a 4-byte header + 8 bytes per track descriptor). bMSF requests
	// MSF addresses (vs LBA). Returns bytes read, or < 0 on error.
	int ReadTOC (void *pBuffer, size_t nBufLen, u8 uchStartTrack, boolean bMSF);

	// MMC READ CD (0xBE) for CD-DA audio: reads nFrames sectors starting at LBA
	// nLBA as raw 2352-byte main-channel frames into pBuffer (must hold
	// nFrames*2352 bytes). Format is 44100 Hz / 16-bit / stereo. Returns bytes
	// read (nFrames*2352), or < 0 on error.
	int ReadCDDA (u32 nLBA, unsigned nFrames, void *pBuffer);

	// MMC SET CD SPEED (0xBB): cap the drive's read speed in kB/s (1x audio is
	// 176 kB/s). A low cap keeps the spindle slow, which reduces noise and --
	// important on a bus-powered drive -- peak current draw. Returns TRUE on
	// success; drives that do not implement the command fail harmlessly.
	boolean SetCDSpeed (u16 usReadSpeedKBs);

	// SCSI START STOP UNIT (0x1B) with LoEj=1: eject the disc / open the tray.
	// Best-effort -- can be slow, and drives without a powered tray fail
	// harmlessly. Returns TRUE on success.
	boolean Eject (void);

	// SPC PREVENT ALLOW MEDIUM REMOVAL (0x1E): lock (bPrevent=TRUE) or unlock the
	// drive's eject mechanism, so removal is host-controlled while media is in use.
	// Best-effort; drives that do not implement it fail harmlessly. Returns TRUE on
	// success.
	boolean PreventAllowRemoval (boolean bPrevent);

	// Recover a wedged Bulk-Only Transport in place, KEEPING the device:
	// BULK_ONLY_MASS_STORAGE_RESET + clear-halt on both bulk pipes (+ Pi4 EP0
	// recovery). Mirrors Linux usb_stor_Bulk_reset -- the right recovery after a
	// transaction error (e.g. an external eject that desyncs the pipe), as opposed
	// to RemoveDevice() which deletes the device and needs a hub connection edge to
	// come back. Returns 0 on success, < 0 if the device is unreachable.
	int Reset (void);

	// ---- Async Bulk-Only Transport (non-blocking, tick-driven) -------------
	//
	// The blocking Command() above parks core 0 for the whole transfer -- on a
	// CD that means the whole ~2.8-4.5 s spin-up of the first read, freezing the
	// UI/VBL/audio. The async path instead submits each BOT phase and returns,
	// so the owner (core 0's service loop) stays live and polls for completion.
	// Same phase shape as Command() / Linux usb_stor_Bulk_transport; only the
	// blocking wait becomes a submit + poll. ONE async command in flight at a
	// time (the caller serializes). See ASYNC_BOT.md.
	enum TCommandAsyncStatus
	{
		CommandAsyncBusy,	// a phase is in flight; call Step() again later
		CommandAsyncDone,	// finished OK; result = GetCommandAsyncResult()
		CommandAsyncFailed,	// ordinary failure (UMSD_CMD_FAILED-class)
		CommandAsyncError	// transaction error (UMSD_CMD_ERROR-class)
	};

	// Begin an async command. Returns FALSE if one is already in flight or the
	// CBW submit fails. pBuffer/nBufLen is the data phase (may be 0). The buffer
	// must stay valid until Step() reports Done/Failed/Error.
	boolean CommandAsyncStart (void *pCmdBlk, size_t nCmdBlkLen,
				   void *pBuffer, size_t nBufLen, boolean bIn);

	// Async READ CD (CD-DA): same CDB as the blocking ReadCDDA(), driven via the
	// async transport. pBuffer must hold nFrames*2352 bytes (and be DMA-safe) and
	// stay valid until CommandAsyncStep() reports Done. Drive Step() to completion;
	// on Done, GetCommandAsyncResult() is the byte count. Returns FALSE if a
	// command is already in flight.
	boolean ReadCDDAAsyncStart (u32 nLBA, unsigned nFrames, void *pBuffer);

	// Async eject (START STOP UNIT, LoEj): same as Eject() but non-blocking, so the
	// caller is not parked for the slow mechanical-eject CSW. Drive CommandAsyncStep()
	// to completion. Returns FALSE if a command is already in flight.
	boolean EjectAsyncStart (void);

	// Advance the in-flight async command. Call from the owner's service tick.
	// Never blocks: returns Busy until the current phase's completion IRQ fires.
	TCommandAsyncStatus CommandAsyncStep (void);

	// Bytes delivered in the data phase (valid after Step() returns Done).
	int GetCommandAsyncResult (void) const { return m_nAsyncResult; }

private:
	int TryRead (void *pBuffer, size_t nCount);
	int TryWrite (const void *pBuffer, size_t nCount);

	int Command (void *pCmdBlk, size_t nCmdBlkLen, void *pBuffer, size_t nBufLen, boolean bIn);

	// Clear a STALL on a bulk endpoint: device-side CLEAR_FEATURE + (on Pi4)
	// host-side xHCI Reset Endpoint. Required so the next bulk transfer runs
	// after a short-data-phase stall. Returns TRUE on success.
	boolean ClearEndpointHalt (CUSBEndpoint *pEndpoint);

	void LogRequestSense (void);

	// Async transport internals (see the public async section above).
	enum TAsyncPhase { AsyncIdle, AsyncCBW, AsyncData, AsyncCSW, AsyncCSW2 };
	boolean AsyncSubmit (CUSBEndpoint *pEndpoint, void *pBuffer, unsigned nBufLen);
	static void AsyncCompletion (CUSBRequest *pURB, void *pParam, void *pContext);

private:
	CUSBEndpoint *m_pEndpointIn;
	CUSBEndpoint *m_pEndpointOut;

	unsigned m_nCWBTag;
	unsigned m_nBlockCount;
	u64 m_ullOffset;

	boolean  m_bCDROM;		// SCSI peripheral device type 0x05
	unsigned m_nBlockSize;		// 512 for disks, 2048 for CD-ROM
	u32	 m_nLastSense;		// last logged sense (key<<16|ASC<<8|ASCQ)

	CPartitionManager *m_pPartitionManager;

	static CNumberPool s_DeviceNumberPool;
	unsigned m_nDeviceNumber;

	// Async BOT state (one command in flight). m_pAsyncCBW/CSW are cache-aligned
	// HEAP_DMA30 buffers allocated once (persist across Step() calls, unlike the
	// stack DMA_BUFFERs in Command()); m_AsyncDone is set by the completion IRQ.
	TAsyncPhase	  m_AsyncPhase;
	CUSBRequest	 *m_pAsyncURB;
	volatile boolean  m_AsyncDone;
	u8		 *m_pAsyncCBW;		// HEAP_DMA30, sizeof(TCBW)
	u8		 *m_pAsyncCSW;		// HEAP_DMA30, sizeof(TCSW)
	void		 *m_pAsyncData;		// caller's data buffer (may be 0)
	unsigned	  m_nAsyncDataLen;
	boolean		  m_bAsyncIn;
	int		  m_nAsyncResult;	// data bytes delivered (Done)
};

#endif
