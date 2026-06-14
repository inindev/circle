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

private:
	int TryRead (void *pBuffer, size_t nCount);
	int TryWrite (const void *pBuffer, size_t nCount);

	int Command (void *pCmdBlk, size_t nCmdBlkLen, void *pBuffer, size_t nBufLen, boolean bIn);

	// Clear a STALL on a bulk endpoint: device-side CLEAR_FEATURE + (on Pi4)
	// host-side xHCI Reset Endpoint. Required so the next bulk transfer runs
	// after a short-data-phase stall. Returns TRUE on success.
	boolean ClearEndpointHalt (CUSBEndpoint *pEndpoint);

	void LogRequestSense (void);

	int Reset (void);

private:
	CUSBEndpoint *m_pEndpointIn;
	CUSBEndpoint *m_pEndpointOut;

	unsigned m_nCWBTag;
	unsigned m_nBlockCount;
	u64 m_ullOffset;

	u32	 m_nLastSense;		// last logged sense (key<<16|ASC<<8|ASCQ)

	CPartitionManager *m_pPartitionManager;

	static CNumberPool s_DeviceNumberPool;
	unsigned m_nDeviceNumber;
};

#endif
