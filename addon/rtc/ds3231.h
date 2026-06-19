//
// ds3231.h
//
// Driver for the Maxim/Analog Devices DS3231 I2C real-time clock.
//
// Circle - A C++ bare metal environment for Raspberry Pi
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
#ifndef _rtc_ds3231_h
#define _rtc_ds3231_h

#include <rtc/rtc.h>
#include <circle/i2cmaster.h>
#include <circle/time.h>
#include <circle/types.h>

class CDS3231 : public CRealTimeClock
{
public:
	CDS3231 (CI2CMaster *pI2CMaster, unsigned nI2CClockHz = 100000, u8 ucSlaveAddress = 0x68);
	~CDS3231 (void);

	// Probe for the device on the I2C bus. Returns FALSE if it does not ACK
	// (e.g. the module is not plugged in), so the caller can treat the RTC as
	// optional.
	boolean Initialize (void);

	boolean Get (CTime *pTime);
	boolean Set (const CTime &Time);

private:
	CI2CMaster *m_pI2CMaster;
	unsigned    m_nI2CClockHz;
	u8	    m_ucSlaveAddress;
};

#endif
