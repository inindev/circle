//
// netdevlayer.h
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2015-2025  R. Stange <rsta2@gmx.net>
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
#ifndef _circle_net_netdevlayer_h
#define _circle_net_netdevlayer_h

#include <circle/net/netconfig.h>
#include <circle/netdevice.h>
#include <circle/net/netbuffer.h>
#include <circle/net/netbufferqueue.h>
#include <circle/bcm54213.h>
#include <circle/macb.h>
#include <circle/types.h>

class CNetDeviceLayer
{
public:
	// pInjectedDevice (optional): bind this CNetDevice instead of constructing
	// and initializing the built-in on-board NIC. When 0 (default), behaves as
	// before -- builds/binds the SoC Ethernet (Bcm54213 / MACB). Supplying a
	// device lets a caller run the IP stack over a virtual/shared NIC (e.g. a
	// demux shim over a NIC that is also used for raw frames) without a second
	// hardware driver instance fighting for the same NIC.
	CNetDeviceLayer (CNetConfig *pNetConfig, TNetDeviceType DeviceType,
			 CNetDevice *pInjectedDevice = 0);
	~CNetDeviceLayer (void);

	boolean Initialize (boolean bWaitForActivate);

	void Process (void);

	// returns 0, if net device is not available yet
	const CMACAddress *GetMACAddress (void) const;

	void Send (CNetBuffer *pNetBuffer);
	CNetBuffer *Receive (void);

	boolean IsRunning (void) const;		// is net device available and link up?

	// terminated with 00:00:00:00:00:00
	boolean SetMulticastFilter (const u8 Groups[][MAC_ADDRESS_SIZE]);

private:
	TNetDeviceType m_DeviceType;
	CNetConfig *m_pNetConfig;
	CNetDevice *m_pDevice;
	CNetDevice *m_pInjectedDevice;		// non-0: use this, skip the built-in NIC

	CNetBufferQueue m_TxQueue;
	CNetBufferQueue m_RxQueue;

	CNetBuffer *m_pRxBuffer;

#if RASPPI == 4
	CBcm54213Device m_Bcm54213;
#elif RASPPI >= 5
	CMACBDevice m_MACB;
#endif
};

#endif
