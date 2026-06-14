//
// usbmassdevice.cpp
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
#include <circle/usb/usbmassdevice.h>
#include <circle/usb/usbhostcontroller.h>
#include <circle/usb/xhciendpoint.h>		// CXHCIEndpoint::ResetFromHalted (Pi4)
#include <circle/devicenameservice.h>
#include <circle/logger.h>
#include <circle/timer.h>
#include <circle/util.h>
#include <circle/synchronize.h>
#include <circle/macros.h>
#include <circle/new.h>
#include <assert.h>

#define MAX_TRIES	8				// max. read / write attempts

// UMSD_CMD_FAILED / UMSD_CMD_ERROR result codes are defined in the header
// (usbmassdevice.h) so callers can distinguish a transaction error.

// USB Mass Storage Bulk-Only Transport

// Class-specific requests
#define GET_MAX_LUN			0xFE
#define BULK_ONLY_MASS_STORAGE_RESET	0xFF

// Command Block Wrapper
struct TCBW
{
	u32		dCWBSignature,
#define CBWSIGNATURE		0x43425355
			dCWBTag,
			dCBWDataTransferLength;		// number of bytes
	u8		bmCBWFlags,
#define CBWFLAGS_DATA_IN	0x80
			bCBWLUN		: 4,
#define CBWLUN			0
			Reserved1	: 4,
			bCBWCBLength	: 5,		// valid length of the CBWCB in bytes
			Reserved2	: 3;
	u8		CBWCB[16];
}
PACKED;

// Command Status Wrapper
struct TCSW
{
	u32		dCSWSignature,
#define CSWSIGNATURE		0x53425355
			dCSWTag,
			dCSWDataResidue;		// difference in amount of data processed
	u8		bCSWStatus;
#define CSWSTATUS_PASSED	0x00
#define CSWSTATUS_FAILED	0x01
#define CSWSTATUS_PHASE_ERROR	0x02
}
PACKED;

// SCSI Transparent Command Set

#define SCSI_CONTROL		0x00

struct TSCSIInquiry
{
	u8		OperationCode,
#define SCSI_OP_INQUIRY		0x12
			LogicalUnitNumberEVPD,
			PageCode,
			Reserved,
			AllocationLength,
			Control;
}
PACKED;

struct TSCSIInquiryResponse
{
	u8		PeripheralDeviceType	: 5,
#define SCSI_PDT_DIRECT_ACCESS_BLOCK	0x00			// SBC-2 command set (or above)
#define SCSI_PDT_CD_DVD			0x05			// MMC command set (CD/DVD)
#define SCSI_PDT_OPTICAL_MEMORY		0x07			// MMC optical memory (DVD+RW etc.)
#define SCSI_PDT_DIRECT_ACCESS_RBC	0x0E			// RBC command set
			PeripheralQualifier	: 3,		// 0: device is connected to this LUN
			DeviceTypeModifier	: 7,
			RMB			: 1,		// 1: removable media
			ANSIApprovedVersion	: 3,
			ECMAVersion		: 3,
			ISOVersion		: 2,
			Reserved1,
			AdditionalLength,
			Reserved2[3],
			VendorIdentification[8],
			ProductIdentification[16],
			ProductRevisionLevel[4];
}
PACKED;

struct TSCSITestUnitReady
{
	u8		OperationCode;
#define SCSI_OP_TEST_UNIT_READY		0x00
	u32		Reserved;
	u8		Control;
}
PACKED;

struct TSCSIRequestSense
{
	u8		OperationCode;
#define SCSI_REQUEST_SENSE		0x03
	u8		DescriptorFormat	: 1,		// set to 0
			Reserved1		: 7;
	u16		Reserved2;
	u8		AllocationLength;
	u8		Control;
}
PACKED;

struct TSCSIRequestSenseResponse7x
{
	u8		ResponseCode		: 7,
			Valid			: 1;
	u8		Obsolete;
	u8		SenseKey		: 4,
			Reserved		: 1,
			ILI			: 1,
			EOM			: 1,
			FileMark		: 1;
	u32		Information;				// big endian
	u8		AdditionalSenseLength;
	u32		CommandSpecificInformation;		// big endian
	u8		AdditionalSenseCode;
	u8		AdditionalSenseCodeQualifier;
	u8		FieldReplaceableUnitCode;
	u8		SenseKeySpecificHigh	: 7,
			SKSV			: 1;
	u16		SenseKeySpecificLow;
}
PACKED;

struct TSCSIReadCapacity10
{
	u8		OperationCode;
#define SCSI_OP_READ_CAPACITY10		0x25
	u8		Obsolete		: 1,
			Reserved1		: 7;
	u32		LogicalBlockAddress;			// set to 0
	u16		Reserved2;
	u8		PartialMediumIndicator	: 1,		// set to 0
			Reserved3		: 7;
	u8		Control;
}
PACKED;

struct TSCSIReadCapacityResponse
{
	u32		ReturnedLogicalBlockAddress;		// big endian
	u32		BlockLengthInBytes;			// big endian
}
PACKED;

struct TSCSIRead10
{
	u8		OperationCode,
#define SCSI_OP_READ		0x28
			Reserved1;
	u32		LogicalBlockAddress;			// big endian
	u8		Reserved2;
	u16		TransferLength;				// block count, big endian
	u8		Control;
#define SCSI_READ_CONTROL	0x00
}
PACKED;

struct TSCSIWrite10
{
	u8		OperationCode,
#define SCSI_OP_WRITE		0x2A
			Flags;
#define SCSI_WRITE_FUA		0x08
	u32		LogicalBlockAddress;			// big endian
	u8		Reserved;
	u16		TransferLength;				// block count, big endian
	u8		Control;
#define SCSI_WRITE_CONTROL	0x00
}
PACKED;

// MMC READ TOC/PMA/ATIP (10) -- returns the disc's table of contents.
struct TSCSIReadTOC
{
	u8		OperationCode;
#define SCSI_OP_READ_TOC	0x43
	u8		MSF;					// bit 1: addresses as MSF
	u8		Format;					// 0 = TOC
	u8		Reserved[3];
	u8		StartingTrack;
	u16		AllocationLength;			// big endian
	u8		Control;
}
PACKED;

// MMC READ CD (12) -- byte layout per MMC Table 34, byte values matched to a
// usbmon capture of Linux reading this drive (byte 9 = 0x10, User Data).
struct TSCSIReadCD
{
	u8		OperationCode;
#define SCSI_OP_READ_CD		0xBE
	u8		SectorType;				// byte 1: expected sector type
#define SCSI_READCD_FMT_CDDA	(0x01 << 2)		// Linux format=1 for CD-DA
	u32		LogicalBlockAddress;			// bytes 2-5, big endian
	u8		TransferLength[3];			// bytes 6-8, 24-bit block count
	u8		Selection;				// byte 9: field selection bits
#define SCSI_READCD_USER_DATA	0x10			// main-channel User Data (2352 B)
	u8		SubChannelSelection;			// byte 10
	u8		Control;				// byte 11
}
PACKED;

// MMC SET CD SPEED (12) -- caps the logical unit's read/write speed in kB/s.
struct TSCSISetCDSpeed
{
	u8		OperationCode;
#define SCSI_OP_SET_CD_SPEED	0xBB
	u8		Reserved1;
	u16		ReadSpeed;				// kB/s, big endian
	u16		WriteSpeed;				// kB/s, big endian
	u8		Reserved2[5];
	u8		Control;
}
PACKED;

// SBC/MMC START STOP UNIT (6) -- spins the medium up/down and, for removable
// media, loads or ejects it.
struct TSCSIStartStopUnit
{
	u8		OperationCode;
#define SCSI_OP_START_STOP_UNIT	0x1B
	u8		Immediate;				// bit 0: return before completion
	u8		Reserved[2];
	u8		LoEjStart;				// bit 0 Start, bit 1 LoEj
#define SCSI_SSU_LOEJ		0x02			// act on the tray (load/eject)
#define SCSI_SSU_START		0x01			// 1 = load/spin up, 0 = eject/stop
	u8		Control;
}
PACKED;

#define CDDA_FRAME_SIZE		2352			// raw audio bytes per CD sector

CNumberPool CUSBBulkOnlyMassStorageDevice::s_DeviceNumberPool (1);

static const char FromUmsd[] = "umsd";

// Like CUSBHostController::Transfer(), but reports the error class via *pError
// (USBErrorStall vs. fatal). The BOT data/status phases must distinguish a
// STALL (a protocol answer, recoverable by clearing the halt) from a
// transaction error or timeout (the device is not responding; clearing the
// halt would just block for another timeout period).
static int TransferWithError (CUSBHostController *pHost, CUSBEndpoint *pEndpoint,
			      void *pBuffer, unsigned nBufLen, TUSBError *pError)
{
	assert (pError != 0);

	CUSBRequest URB (pEndpoint, pBuffer, nBufLen);

	if (!pHost->SubmitBlockingRequest (&URB))
	{
		*pError = URB.GetUSBError ();

		return -1;
	}

	*pError = USBErrorUnknown;

	return URB.GetResultLength ();
}

CUSBBulkOnlyMassStorageDevice::CUSBBulkOnlyMassStorageDevice (CUSBFunction *pFunction)
:	CUSBFunction (pFunction),
	m_pEndpointIn (0),
	m_pEndpointOut (0),
	m_nCWBTag (0),
	m_nBlockCount (0),
	m_ullOffset (0),
	m_bCDROM (FALSE),
	m_nBlockSize (UMSD_BLOCK_SIZE),
	m_nLastSense ((u32) -1),
	m_pPartitionManager (0),
	m_nDeviceNumber (0)
{
}

CUSBBulkOnlyMassStorageDevice::~CUSBBulkOnlyMassStorageDevice (void)
{
	if (m_nDeviceNumber != 0)
	{
		CDeviceNameService::Get ()->RemoveDevice (m_bCDROM ? "ucd" : "umsd",
							  m_nDeviceNumber, TRUE);

		s_DeviceNumberPool.FreeNumber (m_nDeviceNumber);

		m_nDeviceNumber = 0;
	}

	delete m_pPartitionManager;
	m_pPartitionManager = 0;

	delete m_pEndpointOut;
	m_pEndpointOut =  0;
	
	delete m_pEndpointIn;
	m_pEndpointIn = 0;
}

boolean CUSBBulkOnlyMassStorageDevice::Configure (void)
{
	if (GetNumEndpoints () < 2)
	{
		ConfigurationError (FromUmsd);

		return FALSE;
	}

	const TUSBEndpointDescriptor *pEndpointDesc;
	while ((pEndpointDesc = (TUSBEndpointDescriptor *) GetDescriptor (DESCRIPTOR_ENDPOINT)) != 0)
	{
		if ((pEndpointDesc->bmAttributes & 0x3F) == 0x02)		// Bulk
		{
			if ((pEndpointDesc->bEndpointAddress & 0x80) == 0x80)	// Input
			{
				if (m_pEndpointIn != 0)
				{
					ConfigurationError (FromUmsd);

					return FALSE;
				}

				m_pEndpointIn = new CUSBEndpoint (GetDevice (), pEndpointDesc);
			}
			else							// Output
			{
				if (m_pEndpointOut != 0)
				{
					ConfigurationError (FromUmsd);

					return FALSE;
				}

				m_pEndpointOut = new CUSBEndpoint (GetDevice (), pEndpointDesc);
			}
		}
	}

	if (   m_pEndpointIn  == 0
	    || m_pEndpointOut == 0)
	{
		ConfigurationError (FromUmsd);

		return FALSE;
	}

	if (!CUSBFunction::Configure ())
	{
		CLogger::Get ()->Write (FromUmsd, LogError, "Cannot set interface");

		return FALSE;
	}

	TSCSIInquiry SCSIInquiry;
	SCSIInquiry.OperationCode	  = SCSI_OP_INQUIRY;
	SCSIInquiry.LogicalUnitNumberEVPD = 0;
	SCSIInquiry.PageCode		  = 0;
	SCSIInquiry.Reserved		  = 0;
	SCSIInquiry.AllocationLength	  = sizeof (TSCSIInquiryResponse);
	SCSIInquiry.Control		  = SCSI_CONTROL;

	TSCSIInquiryResponse SCSIInquiryResponse;
	if (Command (&SCSIInquiry, sizeof SCSIInquiry,
		     &SCSIInquiryResponse, sizeof SCSIInquiryResponse,
		     TRUE) != (int) sizeof SCSIInquiryResponse)
	{
		CLogger::Get ()->Write (FromUmsd, LogError, "Device does not respond");

		return FALSE;
	}

	// Accept direct-access block devices (disks, PDT 0x00) and CD-ROM/optical
	// devices (PDT 0x05). A USB CD drive enumerates as the same Mass-Storage /
	// SCSI-transparent / Bulk-Only class and uses the same bulk transport; it
	// differs in 2048-byte blocks (handled below) and the MMC command set.
	switch (SCSIInquiryResponse.PeripheralDeviceType)
	{
	case SCSI_PDT_DIRECT_ACCESS_BLOCK:
		m_bCDROM = FALSE;
		break;

	case SCSI_PDT_CD_DVD:
	case SCSI_PDT_OPTICAL_MEMORY:
		m_bCDROM = TRUE;
		break;

	default:
		CLogger::Get ()->Write (FromUmsd, LogError, "Unsupported device type: 0x%02X", (unsigned) SCSIInquiryResponse.PeripheralDeviceType);

		return FALSE;
	}

	// Wait for the unit to become ready. A disk is ready promptly; a CD drive
	// with no disc inserted never becomes ready, which is normal — we must still
	// register the drive so a disc inserted later can be used. So for a CD-ROM,
	// don't fail on "not ready": fall through with no media (m_nBlockCount 0),
	// and let capacity be (re)read on demand when a disc is present.
	unsigned nTries = m_bCDROM ? 30 : 100;	// CD: allow time to spin up, then register anyway
	while (--nTries)
	{
		CTimer::Get ()->MsDelay (100);

		TSCSITestUnitReady SCSITestUnitReady;
		SCSITestUnitReady.OperationCode = SCSI_OP_TEST_UNIT_READY;
		SCSITestUnitReady.Reserved	= 0;
		SCSITestUnitReady.Control	= SCSI_CONTROL;

		if (Command (&SCSITestUnitReady, sizeof SCSITestUnitReady, 0, 0, FALSE) >= 0)
		{
			break;
		}

		TSCSIRequestSense SCSIRequestSense;
		SCSIRequestSense.OperationCode	  = SCSI_REQUEST_SENSE;
		SCSIRequestSense.DescriptorFormat = 0;
		SCSIRequestSense.Reserved1	  = 0;
		SCSIRequestSense.Reserved2	  = 0;
		SCSIRequestSense.AllocationLength = sizeof (TSCSIRequestSenseResponse7x);
		SCSIRequestSense.Control	  = SCSI_CONTROL;

		TSCSIRequestSenseResponse7x SCSIRequestSenseResponse7x;
		if (Command (&SCSIRequestSense, sizeof SCSIRequestSense,
			     &SCSIRequestSenseResponse7x, sizeof SCSIRequestSenseResponse7x,
			     TRUE) < 0)
		{
			if (!m_bCDROM)
			{
				CLogger::Get ()->Write (FromUmsd, LogError, "Request sense failed");

				return FALSE;
			}
		}
	}

	boolean bReady = (nTries != 0);
	if (!bReady && !m_bCDROM)
	{
		CLogger::Get ()->Write (FromUmsd, LogError, "Unit is not ready");

		return FALSE;
	}

	if (bReady)
	{
		TSCSIReadCapacity10 SCSIReadCapacity;
		SCSIReadCapacity.OperationCode		= SCSI_OP_READ_CAPACITY10;
		SCSIReadCapacity.Obsolete		= 0;
		SCSIReadCapacity.Reserved1		= 0;
		SCSIReadCapacity.LogicalBlockAddress	= 0;
		SCSIReadCapacity.Reserved2		= 0;
		SCSIReadCapacity.PartialMediumIndicator	= 0;
		SCSIReadCapacity.Reserved3		= 0;
		SCSIReadCapacity.Control		= SCSI_CONTROL;

		TSCSIReadCapacityResponse SCSIReadCapacityResponse;
		if (Command (&SCSIReadCapacity, sizeof SCSIReadCapacity,
			     &SCSIReadCapacityResponse, sizeof SCSIReadCapacityResponse,
			     TRUE) == (int) sizeof SCSIReadCapacityResponse)
		{
			unsigned nBlockSize = le2be32 (SCSIReadCapacityResponse.BlockLengthInBytes);
			// Disks use 512-byte blocks; CD-ROM media reports 2048. Accept the
			// device's reported size (stored per-instance) rather than hardcoding.
			if (nBlockSize != 512 && nBlockSize != 2048)
			{
				if (!m_bCDROM)
				{
					CLogger::Get ()->Write (FromUmsd, LogError,
								"Unsupported block size: %u", nBlockSize);

					return FALSE;
				}
			}
			else
			{
				m_nBlockSize = nBlockSize;
			}

			m_nBlockCount = le2be32 (SCSIReadCapacityResponse.ReturnedLogicalBlockAddress);
			if (m_nBlockCount == (u32) -1)
			{
				if (!m_bCDROM)
				{
					CLogger::Get ()->Write (FromUmsd, LogError,
								"Unsupported disk size > 2TB");

					return FALSE;
				}

				m_nBlockCount = 0;
			}
			else
			{
				m_nBlockCount++;
			}
		}
		else if (!m_bCDROM)
		{
			CLogger::Get ()->Write (FromUmsd, LogError, "Read capacity failed");

			return FALSE;
		}
		else
		{
			// Pure audio CD: no data capacity, tracks come from the TOC.
			m_nBlockSize = 2048;
			m_nBlockCount = 0;
		}
	}
	else
	{
		// CD drive with no disc: register anyway, no media yet.
		m_nBlockSize = 2048;
		m_nBlockCount = 0;
	}

	CLogger::Get ()->Write (FromUmsd, LogNotice, "%s: %u blocks of %u bytes",
				m_bCDROM ? "CD-ROM" : "disk", m_nBlockCount, m_nBlockSize);

	unsigned nDeviceNumber = s_DeviceNumberPool.AllocateNumber (FALSE);
	if (nDeviceNumber == CNumberPool::Invalid)
	{
		CLogger::Get ()->Write (FromUmsd, LogError, "Too many devices");

		return FALSE;
	}

	assert (m_nDeviceNumber == 0);
	m_nDeviceNumber = nDeviceNumber;

	// CD-ROM registers as "ucd<N>"; a disk as "umsd<N>". CDs have no PC-style
	// partition map (and no media may be present), so skip the partition
	// manager for optical devices.
	CString DeviceName;
	DeviceName.Format (m_bCDROM ? "ucd%u" : "umsd%u", m_nDeviceNumber);

	if (!m_bCDROM)
	{
		assert (m_pPartitionManager == 0);
		m_pPartitionManager = new CPartitionManager (this, DeviceName);
		assert (m_pPartitionManager != 0);
		if (!m_pPartitionManager->Initialize ())
		{
			s_DeviceNumberPool.FreeNumber (m_nDeviceNumber);
			m_nDeviceNumber = 0;

			return FALSE;
		}
	}

	CDeviceNameService::Get ()->AddDevice (DeviceName, this, TRUE);
	
	return TRUE;
}

int CUSBBulkOnlyMassStorageDevice::Read (void *pBuffer, size_t nCount)
{
	unsigned nTries = MAX_TRIES;

	int nResult;

	do
	{
		nResult = TryRead (pBuffer, nCount);

		if (nResult != (int) nCount)
		{
			int nStatus = Reset ();
			if (nStatus != 0)
			{
				return nStatus;
			}
		}
	}
	while (   nResult != (int) nCount
	       && --nTries > 0);

	return nResult;
}

int CUSBBulkOnlyMassStorageDevice::Write (const void *pBuffer, size_t nCount)
{
	unsigned nTries = MAX_TRIES;

	int nResult;

	do
	{
		nResult = TryWrite (pBuffer, nCount);

		if (nResult != (int) nCount)
		{
			int nStatus = Reset ();
			if (nStatus != 0)
			{
				return nStatus;
			}
		}
	}
	while (   nResult != (int) nCount
	       && --nTries > 0);

	return nResult;
}

u64 CUSBBulkOnlyMassStorageDevice::Seek (u64 ullOffset)
{
	m_ullOffset = ullOffset;

	return m_ullOffset;
}

u64 CUSBBulkOnlyMassStorageDevice::GetSize (void) const
{
	assert (m_nBlockCount < (u32) -1);

	return (u64) m_nBlockCount * m_nBlockSize;
}

unsigned CUSBBulkOnlyMassStorageDevice::GetCapacity (void) const
{
	return m_nBlockCount;
}

boolean CUSBBulkOnlyMassStorageDevice::RefreshMedia (void)
{
	// TEST UNIT READY: is media present and spun up?
	TSCSITestUnitReady SCSITestUnitReady;
	SCSITestUnitReady.OperationCode = SCSI_OP_TEST_UNIT_READY;
	SCSITestUnitReady.Reserved	= 0;
	SCSITestUnitReady.Control	= SCSI_CONTROL;

	if (Command (&SCSITestUnitReady, sizeof SCSITestUnitReady, 0, 0, FALSE) < 0)
	{
		// Not ready (no disc, or spinning up). Clear any stale capacity.
		m_nBlockCount = 0;
		return FALSE;
	}

	// Ready: refresh data capacity. (A pure audio disc has no addressable data
	// blocks and may report 0 / fail here — that's fine; its tracks come from
	// the TOC, not READ CAPACITY.)
	TSCSIReadCapacity10 SCSIReadCapacity;
	SCSIReadCapacity.OperationCode		= SCSI_OP_READ_CAPACITY10;
	SCSIReadCapacity.Obsolete		= 0;
	SCSIReadCapacity.Reserved1		= 0;
	SCSIReadCapacity.LogicalBlockAddress	= 0;
	SCSIReadCapacity.Reserved2		= 0;
	SCSIReadCapacity.PartialMediumIndicator	= 0;
	SCSIReadCapacity.Reserved3		= 0;
	SCSIReadCapacity.Control		= SCSI_CONTROL;

	TSCSIReadCapacityResponse SCSIReadCapacityResponse;
	if (Command (&SCSIReadCapacity, sizeof SCSIReadCapacity,
		     &SCSIReadCapacityResponse, sizeof SCSIReadCapacityResponse,
		     TRUE) == (int) sizeof SCSIReadCapacityResponse)
	{
		unsigned nBlockSize = le2be32 (SCSIReadCapacityResponse.BlockLengthInBytes);
		if (nBlockSize == 512 || nBlockSize == 2048)
			m_nBlockSize = nBlockSize;
		m_nBlockCount = le2be32 (SCSIReadCapacityResponse.ReturnedLogicalBlockAddress);
		if (m_nBlockCount != (u32) -1)
			m_nBlockCount++;
		else
			m_nBlockCount = 0;
	}
	else
	{
		m_nBlockCount = 0;		// audio-only disc or no data capacity
	}

	return TRUE;
}

int CUSBBulkOnlyMassStorageDevice::ReadTOC (void *pBuffer, size_t nBufLen,
					   u8 uchStartTrack, boolean bMSF)
{
	assert (pBuffer != 0);

	// Request the full buffer; the device returns only as much TOC as exists and
	// (per the BOT spec) may stall the IN endpoint to terminate the short
	// response. Command() handles that stall and reports the actual bytes
	// received via the CSW data residue -- so over-requesting is fine.
	TSCSIReadTOC SCSIReadTOC;
	SCSIReadTOC.OperationCode  = SCSI_OP_READ_TOC;
	SCSIReadTOC.MSF		   = bMSF ? 0x02 : 0x00;		// bit 1 = MSF
	SCSIReadTOC.Format	   = 0;				// 0 = TOC
	SCSIReadTOC.Reserved[0]	   = 0;
	SCSIReadTOC.Reserved[1]	   = 0;
	SCSIReadTOC.Reserved[2]	   = 0;
	SCSIReadTOC.StartingTrack  = uchStartTrack;
	SCSIReadTOC.AllocationLength = le2be16 ((u16) nBufLen);
	SCSIReadTOC.Control	   = SCSI_CONTROL;

	int nResult = Command (&SCSIReadTOC, sizeof SCSIReadTOC, pBuffer, nBufLen, TRUE);
	if (nResult < 0)
		CLogger::Get ()->Write (FromUmsd, LogWarning, "READ TOC failed");

	return nResult;		// bytes actually returned by the device
}

int CUSBBulkOnlyMassStorageDevice::ReadCDDA (u32 nLBA, unsigned nFrames, void *pBuffer)
{
	assert (pBuffer != 0);
	assert (nFrames > 0);

	// READ CD (0xBE) for CD-DA: 2352-byte raw frames per Linux cdrom_read_cdda_old().
	size_t nBufLen = (size_t) nFrames * CDDA_FRAME_SIZE;

	TSCSIReadCD SCSIReadCD;
	memset (&SCSIReadCD, 0, sizeof SCSIReadCD);
	SCSIReadCD.OperationCode      = SCSI_OP_READ_CD;
	SCSIReadCD.SectorType	      = SCSI_READCD_FMT_CDDA;
	SCSIReadCD.LogicalBlockAddress = le2be32 (nLBA);
	SCSIReadCD.TransferLength[0]  = (u8) (nFrames >> 16);		// 24-bit big endian
	SCSIReadCD.TransferLength[1]  = (u8) (nFrames >> 8);
	SCSIReadCD.TransferLength[2]  = (u8) (nFrames);
	SCSIReadCD.Selection	      = SCSI_READCD_USER_DATA;
	SCSIReadCD.SubChannelSelection = 0;
	SCSIReadCD.Control	      = SCSI_CONTROL;

	// Retry with BOT Reset Recovery in between, like Read() does for disks.
	// CD-DA reads are long bulk-IN transfers on a spinning drive; a transient
	// ORDINARY failure (bad sector, drive busy) must not end playback. But a
	// TRANSACTION ERROR (UMSD_CMD_ERROR) means the device is not responding --
	// soft reset cannot recover it (Linux: USB_STOR_TRANSPORT_ERROR), and looping
	// the reset just blocks. Stop immediately and propagate UMSD_CMD_ERROR so the
	// owner can escalate to a port reset.
	unsigned nTries = MAX_TRIES;
	do
	{
		int nResult = Command (&SCSIReadCD, sizeof SCSIReadCD, pBuffer, nBufLen, TRUE);
		if (nResult == (int) nBufLen)
		{
			return nResult;		// bytes read = nFrames * 2352
		}

		if (nResult == UMSD_CMD_ERROR)
		{
			return UMSD_CMD_ERROR;	// not recoverable here -- owner port-resets
		}

		if (Reset () != 0)
		{
			break;			// device unreachable, retrying is pointless
		}
	}
	while (--nTries > 0);

	CLogger::Get ()->Write (FromUmsd, LogWarning, "READ CD (audio) failed");

	return UMSD_CMD_FAILED;
}

boolean CUSBBulkOnlyMassStorageDevice::SetCDSpeed (u16 usReadSpeedKBs)
{
	TSCSISetCDSpeed SCSISetCDSpeed;
	memset (&SCSISetCDSpeed, 0, sizeof SCSISetCDSpeed);
	SCSISetCDSpeed.OperationCode = SCSI_OP_SET_CD_SPEED;
	SCSISetCDSpeed.ReadSpeed     = le2be16 (usReadSpeedKBs);
	SCSISetCDSpeed.WriteSpeed    = le2be16 (0xFFFF);	// don't restrict writing
	SCSISetCDSpeed.Control	     = SCSI_CONTROL;

	return Command (&SCSISetCDSpeed, sizeof SCSISetCDSpeed, 0, 0, FALSE) >= 0;
}

boolean CUSBBulkOnlyMassStorageDevice::Eject (void)
{
	// START STOP UNIT with LoEj=1, Start=0: open the tray / eject the disc. The
	// drive may take a moment and answer the CSW only once the mechanism settles,
	// so this can be slow; the caller treats it as best-effort.
	TSCSIStartStopUnit SCSIStartStopUnit;
	memset (&SCSIStartStopUnit, 0, sizeof SCSIStartStopUnit);
	SCSIStartStopUnit.OperationCode = SCSI_OP_START_STOP_UNIT;
	SCSIStartStopUnit.LoEjStart	= SCSI_SSU_LOEJ;	// LoEj=1, Start=0 -> eject
	SCSIStartStopUnit.Control	= SCSI_CONTROL;

	// Media is now (being) removed: drop the stale data capacity.
	m_nBlockCount = 0;

	return Command (&SCSIStartStopUnit, sizeof SCSIStartStopUnit, 0, 0, FALSE) >= 0;
}

int CUSBBulkOnlyMassStorageDevice::TryRead (void *pBuffer, size_t nCount)
{
	assert (pBuffer != 0);

	// Offset/count are in bytes and must be a whole number of (per-instance)
	// blocks: 512 for disks, 2048 for CD-ROM media.
	if (   (m_ullOffset % m_nBlockSize) != 0
	    || m_ullOffset > UMSD_MAX_OFFSET)
	{
		return -1;
	}
	u32 nBlockAddress = (u32) (m_ullOffset / m_nBlockSize);

	if ((nCount % m_nBlockSize) != 0)
	{
		return -1;
	}
	u16 usTransferLength = (u16) (nCount / m_nBlockSize);

	//CLogger::Get ()->Write (FromUmsd, LogDebug, "TryRead %u/0x%X/%u", nBlockAddress, (unsigned) pBuffer, (unsigned) usTransferLength);

	TSCSIRead10 SCSIRead;
	SCSIRead.OperationCode		= SCSI_OP_READ;
	SCSIRead.Reserved1		= 0;
	SCSIRead.LogicalBlockAddress	= le2be32 (nBlockAddress);
	SCSIRead.Reserved2		= 0;
	SCSIRead.TransferLength		= le2be16 (usTransferLength);
	SCSIRead.Control		= SCSI_CONTROL;

	if (Command (&SCSIRead, sizeof SCSIRead, pBuffer, nCount, TRUE) != (int) nCount)
	{
		CLogger::Get ()->Write (FromUmsd, LogError, "TryRead failed");

		return -1;
	}

	return nCount;
}

int CUSBBulkOnlyMassStorageDevice::TryWrite (const void *pBuffer, size_t nCount)
{
	assert (pBuffer != 0);

	if (   (m_ullOffset % m_nBlockSize) != 0
	    || m_ullOffset > UMSD_MAX_OFFSET)
	{
		return -1;
	}
	u32 nBlockAddress = (u32) (m_ullOffset / m_nBlockSize);

	if ((nCount % m_nBlockSize) != 0)
	{
		return -1;
	}
	u16 usTransferLength = (u16) (nCount / m_nBlockSize);

	//CLogger::Get ()->Write (FromUmsd, LogDebug, "TryWrite %u/0x%X/%u", nBlockAddress, (unsigned) pBuffer, (unsigned) usTransferLength);

	TSCSIWrite10 SCSIWrite;
	SCSIWrite.OperationCode		= SCSI_OP_WRITE;
	SCSIWrite.Flags			= SCSI_WRITE_FUA;
	SCSIWrite.LogicalBlockAddress	= le2be32 (nBlockAddress);
	SCSIWrite.Reserved		= 0;
	SCSIWrite.TransferLength	= le2be16 (usTransferLength);
	SCSIWrite.Control		= SCSI_CONTROL;

	if (Command (&SCSIWrite, sizeof SCSIWrite, (void *) pBuffer, nCount, FALSE) < 0)
	{
		CLogger::Get ()->Write (FromUmsd, LogError, "TryWrite failed");

		return -1;
	}

	return nCount;
}

void CUSBBulkOnlyMassStorageDevice::LogRequestSense (void)
{
	TSCSIRequestSense SCSIRequestSense;
	SCSIRequestSense.OperationCode	  = SCSI_REQUEST_SENSE;
	SCSIRequestSense.DescriptorFormat = 0;
	SCSIRequestSense.Reserved1	  = 0;
	SCSIRequestSense.Reserved2	  = 0;
	SCSIRequestSense.AllocationLength = sizeof (TSCSIRequestSenseResponse7x);
	SCSIRequestSense.Control	  = SCSI_CONTROL;

	TSCSIRequestSenseResponse7x Response;
	if (Command (&SCSIRequestSense, sizeof SCSIRequestSense,
		     &Response, sizeof Response, TRUE) < 0)
	{
		return;
	}

	// Polling loops (e.g. TEST UNIT READY while a disc spins up) yield the same
	// sense many times per second; log only when it changes.
	u32 nSense =   (u32) Response.SenseKey << 16
		     | (u32) Response.AdditionalSenseCode << 8
		     | Response.AdditionalSenseCodeQualifier;
	if (nSense == m_nLastSense)
	{
		return;
	}
	m_nLastSense = nSense;

	CLogger::Get ()->Write (FromUmsd, LogDebug,
				"Sense key 0x%02X ASC 0x%02X ASCQ 0x%02X",
				(unsigned) Response.SenseKey,
				(unsigned) Response.AdditionalSenseCode,
				(unsigned) Response.AdditionalSenseCodeQualifier);
}

// Clear a halt (STALL) condition on a bulk endpoint. Two parts are required:
//   1. CLEAR_FEATURE(ENDPOINT_HALT) tells the *device* to un-halt and reset its
//      data toggle (USB 2.0 9.4.5). ResetPID() mirrors that on our side.
//   2. On xHCI (Pi4) the *host controller* keeps its own per-endpoint state: a
//      STALL moves the endpoint to the Halted state, and no further transfers
//      run until a Reset Endpoint command + Set TR Dequeue Pointer are issued.
//      CLEAR_FEATURE alone does NOT do this -- the next Transfer() just times
//      out (3s) waiting on a ring the controller will not run. CXHCIEndpoint::
//      ResetFromHalted() issues exactly those two TRBs. This is the same
//      recovery CUSBFloppyDiskDevice uses (usbfloppydevice.cpp). Without it,
//      READ TOC / other short-response commands stall the data phase and then
//      hang the CSW read. Returns TRUE on success.
boolean CUSBBulkOnlyMassStorageDevice::ClearEndpointHalt (CUSBEndpoint *pEndpoint)
{
	assert (pEndpoint != 0);

	CUSBHostController *pHost = GetHost ();
	assert (pHost != 0);

	boolean bIn = pEndpoint->IsDirectionIn ();

	if (pHost->ControlMessage (GetEndpoint0 (),
				   REQUEST_TO_ENDPOINT | REQUEST_OUT, CLEAR_FEATURE,
				   ENDPOINT_HALT,
				   pEndpoint->GetNumber () | (bIn ? 0x80 : 0x00), 0, 0) < 0)
	{
		return FALSE;
	}

	pEndpoint->ResetPID ();

#if RASPPI >= 4
	// Host-side xHCI recovery: take the endpoint out of Halted and re-point its
	// transfer ring. Required for the next bulk transfer (the CSW) to run.
	if (!pEndpoint->GetXHCIEndpoint ()->ResetFromHalted ())
	{
		return FALSE;
	}
#endif

	return TRUE;
}

int CUSBBulkOnlyMassStorageDevice::Command (void *pCmdBlk, size_t nCmdBlkLen,
					    void *pBuffer, size_t nBufLen, boolean bIn)
{
	assert (pCmdBlk != 0);
	assert (6 <= nCmdBlkLen && nCmdBlkLen <= 16);
	assert (nBufLen == 0 || pBuffer != 0);

	DMA_BUFFER (u8, CBWBuffer, sizeof (TCBW));
	TCBW *pCBW = (TCBW *) CBWBuffer;
	memset (pCBW, 0, sizeof *pCBW);

	pCBW->dCWBSignature	     = CBWSIGNATURE;
	pCBW->dCWBTag		     = ++m_nCWBTag;
	pCBW->dCBWDataTransferLength = nBufLen;
	pCBW->bmCBWFlags	     = bIn ? CBWFLAGS_DATA_IN : 0;
	pCBW->bCBWLUN		     = CBWLUN;
	pCBW->bCBWCBLength	     = (u8) nCmdBlkLen;

	memcpy (pCBW->CBWCB, pCmdBlk, nCmdBlkLen);

	CUSBHostController *pHost = GetHost ();
	assert (pHost != 0);

	if (pHost->Transfer (m_pEndpointOut, pCBW, sizeof *pCBW) < 0)
	{
		CLogger::Get ()->Write (FromUmsd, LogError, "CBW transfer failed");

		// The device did not accept the command block: a transaction error,
		// not a recoverable stall. Signal escalation (port reset).
		return UMSD_CMD_ERROR;
	}

	// --- Data phase ---------------------------------------------------------
	// BOT case 13 (Hi > Di): variable-length IN responses (READ TOC, INQUIRY)
	// may end with a bulk STALL. That is not an error -- clear the halt and let
	// the CSW report the actual byte count via dCSWDataResidue. Mirrors Linux
	// usb_stor_Bulk_transport() + usb_stor_clear_halt().
	if (nBufLen > 0)
	{
		assert (pBuffer != 0);

		u8 *pDMABuffer = 0;
		if (!IS_CACHE_ALIGNED (pBuffer, nBufLen))
		{
			pDMABuffer = new (HEAP_DMA30) u8[nBufLen];
			assert (pDMABuffer != 0);

			if (!bIn)
			{
				memcpy (pDMABuffer, pBuffer, nBufLen);
			}
		}

		TUSBError Error;
		int nData = TransferWithError (pHost, bIn ? m_pEndpointIn : m_pEndpointOut,
					       pDMABuffer != 0 ? pDMABuffer : pBuffer, nBufLen,
					       &Error);
		if (nData < 0 && Error != USBErrorStall)
		{
			// Transaction error or timeout: the device is not answering on
			// the bus. Abort without touching the pipes (Linux:
			// USB_STOR_XFER_ERROR -> USB_STOR_TRANSPORT_ERROR, no CSW read);
			// the owner escalates to a port reset (clearing halts won't help).
			CLogger::Get ()->Write (FromUmsd, LogWarning, "Data transfer failed");

			delete [] pDMABuffer;

			return UMSD_CMD_ERROR;
		}

		if (nData < 0)
		{
			// STALL terminating a short data phase (BOT case Hi > Di): clear
			// the halt and let the CSW report the actual byte count. If the
			// halt cannot be cleared, the pipe is still wedged host-side --
			// reading the CSW on it would just burn the full transfer timeout
			// before failing. Match Linux usb_stor_Bulk_transport (data-phase
			// clear_halt failure -> USB_STOR_XFER_ERROR, no CSW read) and
			// escalate to a port reset instead.
			if (!ClearEndpointHalt (bIn ? m_pEndpointIn : m_pEndpointOut))
			{
				CLogger::Get ()->Write (FromUmsd, LogWarning,
							"Cannot clear halt after data STALL");

				delete [] pDMABuffer;

				return UMSD_CMD_ERROR;
			}
		}

		if (pDMABuffer != 0)
		{
			if (bIn)
			{
				memcpy (pBuffer, pDMABuffer, nBufLen);
			}

			delete [] pDMABuffer;
		}
	}

	// --- Status phase (get CSW) --------------------------------------------
	DMA_BUFFER (u8, CSWBuffer, sizeof (TCSW));
	TCSW *pCSW = (TCSW *) CSWBuffer;

	TUSBError CSWError;
	int nCSW = TransferWithError (pHost, m_pEndpointIn, pCSW, sizeof *pCSW, &CSWError);
	if (nCSW != (int) sizeof *pCSW)
	{
		// Retry once (Linux: "Attempting to get CSW (2nd try)") -- but only
		// after a STALL (clear the halt first) or a short read. A transaction
		// error / timeout means the device is gone; retrying just blocks again,
		// so signal escalation (port reset) instead.
		if (nCSW < 0 && CSWError != USBErrorStall)
		{
			CLogger::Get ()->Write (FromUmsd, LogError, "CSW transfer failed");

			return UMSD_CMD_ERROR;
		}

		if (   nCSW < 0
		    && !ClearEndpointHalt (m_pEndpointIn))
		{
			// The halt would not clear -- the endpoint is wedged beyond a soft
			// recovery; escalate to a port reset.
			CLogger::Get ()->Write (FromUmsd, LogDebug,
						"Cannot clear halt on endpoint IN");

			return UMSD_CMD_ERROR;
		}

		nCSW = pHost->Transfer (m_pEndpointIn, pCSW, sizeof *pCSW);
		if (nCSW != (int) sizeof *pCSW)
		{
			CLogger::Get ()->Write (FromUmsd, LogError, "CSW transfer failed");

			return UMSD_CMD_ERROR;
		}
	}

	if (pCSW->dCSWSignature != CSWSIGNATURE)
	{
		// Got a response, but it is not a CSW: the BOT transport is desynced
		// (a stale/late response from a prior transfer). A soft retry cannot
		// resync the tag stream; escalate to a port reset.
		CLogger::Get ()->Write (FromUmsd, LogError, "CSW signature is wrong");

		return UMSD_CMD_ERROR;
	}

	if (pCSW->dCSWTag != m_nCWBTag)
	{
		// Wrong tag: also a desynced transport (this CSW belongs to a different
		// command). Escalate to a port reset to resync.
		CLogger::Get ()->Write (FromUmsd, LogError, "CSW tag is wrong");

		return UMSD_CMD_ERROR;
	}

	if (pCSW->bCSWStatus != CSWSTATUS_PASSED)
	{
		// The device answered and reported the command failed (CHECK CONDITION).
		// An ordinary failure, not a transport error -- no port reset needed.
		LogRequestSense ();

		return UMSD_CMD_FAILED;
	}

	u32 nResidue = pCSW->dCSWDataResidue;
	if (nResidue > (u32) nBufLen)
	{
		nResidue = (u32) nBufLen;
	}

	return (int) (nBufLen - nResidue);
}

int CUSBBulkOnlyMassStorageDevice::Reset (void)
{
	CUSBHostController *pHost = GetHost ();
	assert (pHost != 0);
	
	if (pHost->ControlMessage (GetEndpoint0 (),
				   REQUEST_CLASS | REQUEST_TO_INTERFACE | REQUEST_OUT,
				   BULK_ONLY_MASS_STORAGE_RESET, 0, GetInterfaceNumber (), 0, 0) < 0)
	{
#if RASPPI >= 4
		// A transaction error on the control pipe halts EP0 in the xHCI
		// controller as well; no control transfer runs again until a Reset
		// Endpoint + Set TR Dequeue Pointer. Recover EP0 and retry once.
		if (   !GetEndpoint0 ()->GetXHCIEndpoint ()->ResetFromHalted ()
		    || pHost->ControlMessage (GetEndpoint0 (),
					      REQUEST_CLASS | REQUEST_TO_INTERFACE | REQUEST_OUT,
					      BULK_ONLY_MASS_STORAGE_RESET, 0,
					      GetInterfaceNumber (), 0, 0) < 0)
#endif
		{
			CLogger::Get ()->Write (FromUmsd, LogDebug, "Cannot reset device");

			return -1;
		}
	}

	CTimer::Get ()->MsDelay (100);

	if (!ClearEndpointHalt (m_pEndpointIn))
	{
		CLogger::Get ()->Write (FromUmsd, LogDebug, "Cannot clear halt on endpoint IN");

		return -1;
	}

	if (!ClearEndpointHalt (m_pEndpointOut))
	{
		CLogger::Get ()->Write (FromUmsd, LogDebug, "Cannot clear halt on endpoint OUT");

		return -1;
	}

	return 0;
}
