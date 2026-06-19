//
// ds3231.cpp
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
#include <rtc/ds3231.h>
#include <circle/logger.h>
#include <circle/util.h>
#include <assert.h>

// Timekeeping registers (DS3231 datasheet, Table 1)
#define DS3231_REG_SECONDS	0x00
#define DS3231_REG_MINUTES	0x01
#define DS3231_REG_HOURS	0x02
	#define DS3231_BIT_12		0x40	// 1 = 12-hour mode (we require 24-hour)
#define DS3231_REG_DAY		0x03	// day of week, 1..7
#define DS3231_REG_DATE		0x04
#define DS3231_REG_MONTH	0x05
	#define DS3231_BIT_CENTURY	0x80	// toggled when the year wraps 99->00
#define DS3231_REG_YEAR		0x06

#define DS3231_REG_STATUS	0x0F
	#define DS3231_BIT_OSF		0x80	// Oscillator Stop Flag: time is invalid

#define BCD2DEC(val)		(((val) & 0x0F) + ((val) >> 4) * 10)
#define DEC2BCD(val)		((((val) / 10) << 4) + (val) % 10)

const char FromDS3231[] = "ds3231";

CDS3231::CDS3231 (CI2CMaster *pI2CMaster, unsigned nI2CClockHz, u8 ucSlaveAddress)
:	m_pI2CMaster (pI2CMaster),
	m_nI2CClockHz (nI2CClockHz),
	m_ucSlaveAddress (ucSlaveAddress)
{
}

CDS3231::~CDS3231 (void)
{
	m_pI2CMaster = 0;
}

boolean CDS3231::Initialize (void)
{
	assert (m_pI2CMaster != 0);
	m_pI2CMaster->SetClock (m_nI2CClockHz);

	// A zero-length write probes for an ACK at the slave address. If the module
	// is not plugged in, the bus NAKs and this fails -- the caller then treats the
	// RTC as absent (optional hardware).
	if (m_pI2CMaster->Write (m_ucSlaveAddress, 0, 0) != 0)
	{
		CLogger::Get ()->Write (FromDS3231, LogNotice, "Not present (addr 0x%02X)",
					(unsigned) m_ucSlaveAddress);

		return FALSE;
	}

	return TRUE;
}

boolean CDS3231::Get (CTime *pTime)
{
	assert (m_pI2CMaster != 0);
	m_pI2CMaster->SetClock (m_nI2CClockHz);

	// Read the status register first: the Oscillator Stop Flag (OSF) is set when
	// the oscillator has stopped (no/dead backup battery, or first power-up before
	// the time was ever set). In that case the timekeeping registers hold garbage,
	// so report failure and let the caller fall back to its other clock source.
	u8 StatusCmd[] = {DS3231_REG_STATUS};
	int nResult = m_pI2CMaster->Write (m_ucSlaveAddress, StatusCmd, sizeof StatusCmd);
	if (nResult != sizeof StatusCmd)
	{
		CLogger::Get ()->Write (FromDS3231, LogWarning, "I2C write failed (err %d)", nResult);

		return FALSE;
	}

	u8 ucStatus;
	nResult = m_pI2CMaster->Read (m_ucSlaveAddress, &ucStatus, sizeof ucStatus);
	if (nResult != sizeof ucStatus)
	{
		CLogger::Get ()->Write (FromDS3231, LogWarning, "I2C read failed (err %d)", nResult);

		return FALSE;
	}

	if (ucStatus & DS3231_BIT_OSF)
	{
		CLogger::Get ()->Write (FromDS3231, LogNotice,
					"Oscillator stopped (OSF) -- time not valid");

		return FALSE;
	}

	// Read the seven timekeeping registers (0x00..0x06).
	u8 Cmd[] = {DS3231_REG_SECONDS};
	nResult = m_pI2CMaster->Write (m_ucSlaveAddress, Cmd, sizeof Cmd);
	if (nResult != sizeof Cmd)
	{
		CLogger::Get ()->Write (FromDS3231, LogWarning, "I2C write failed (err %d)", nResult);

		return FALSE;
	}

	u8 Reg[7];
	nResult = m_pI2CMaster->Read (m_ucSlaveAddress, Reg, sizeof Reg);
	if (nResult != sizeof Reg)
	{
		CLogger::Get ()->Write (FromDS3231, LogWarning, "I2C read failed (err %d)", nResult);

		return FALSE;
	}

	if (Reg[DS3231_REG_HOURS] & DS3231_BIT_12)
	{
		CLogger::Get ()->Write (FromDS3231, LogWarning, "RTC runs in 12 hours mode");

		return FALSE;
	}

	assert (pTime != 0);
	if (!pTime->SetTime (BCD2DEC (Reg[DS3231_REG_HOURS]   & 0x3F),
			     BCD2DEC (Reg[DS3231_REG_MINUTES] & 0x7F),
			     BCD2DEC (Reg[DS3231_REG_SECONDS] & 0x7F)))
	{
		CLogger::Get ()->Write (FromDS3231, LogWarning, "Invalid time read");

		return FALSE;
	}

	// Year register is 00..99; the century bit in the month register extends it.
	unsigned nYear = BCD2DEC (Reg[DS3231_REG_YEAR]) + 2000;
	if (Reg[DS3231_REG_MONTH] & DS3231_BIT_CENTURY)
		nYear += 100;

	if (!pTime->SetDate (BCD2DEC (Reg[DS3231_REG_DATE]        & 0x3F),
			     BCD2DEC (Reg[DS3231_REG_MONTH] & 0x1F),
			     nYear))
	{
		CLogger::Get ()->Write (FromDS3231, LogWarning, "Invalid date read");

		return FALSE;
	}

	return TRUE;
}

boolean CDS3231::Set (const CTime &Time)
{
	unsigned nYear = Time.GetYear ();

	u8 Reg[7];
	Reg[DS3231_REG_SECONDS] = DEC2BCD ((u8) Time.GetSeconds ()     & 0x7F);
	Reg[DS3231_REG_MINUTES] = DEC2BCD ((u8) Time.GetMinutes ()     & 0x7F);
	Reg[DS3231_REG_HOURS]   = DEC2BCD ((u8) Time.GetHours ()       & 0x3F);	// 24h
	// CTime::GetWeekDay() is 0=Sunday..6=Saturday; DS3231 wants 1=Sunday..7, so +1.
	Reg[DS3231_REG_DAY]     = DEC2BCD ((u8) (Time.GetWeekDay ()+1) & 0x07);
	Reg[DS3231_REG_DATE]    = DEC2BCD ((u8) Time.GetMonthDay ()    & 0x3F);
	Reg[DS3231_REG_MONTH]   = DEC2BCD ((u8) Time.GetMonth ()       & 0x1F);
	Reg[DS3231_REG_YEAR]    = DEC2BCD ((u8) ((nYear - 2000) % 100) & 0xFF);
	if (nYear >= 2100)
		Reg[DS3231_REG_MONTH] |= DS3231_BIT_CENTURY;

	u8 Data[8] = {DS3231_REG_SECONDS};
	memcpy (Data+1, Reg, sizeof Reg);

	assert (m_pI2CMaster != 0);
	m_pI2CMaster->SetClock (m_nI2CClockHz);

	unsigned nResult = m_pI2CMaster->Write (m_ucSlaveAddress, Data, sizeof Data);
	if (nResult != sizeof Data)
	{
		CLogger::Get ()->Write (FromDS3231, LogWarning, "I2C write failed (err %d)", nResult);

		return FALSE;
	}

	// Writing the time clears the Oscillator Stop Flag so a subsequent Get() trusts
	// the registers. Read-modify-write the status register, clearing only OSF and
	// preserving the other bits (alarm flags A1F/A2F, EN32kHz). The RMW is safe
	// here -- the Pi is the sole I2C master and CI2CMaster serializes access, so
	// there is no second-master race. A failure to clear OSF is non-fatal: the time
	// registers are still valid; the next Get() would just report OSF and fall back.
	u8 StatusCmd[] = {DS3231_REG_STATUS};
	if (m_pI2CMaster->Write (m_ucSlaveAddress, StatusCmd, sizeof StatusCmd) == sizeof StatusCmd)
	{
		u8 ucStatus;
		if (m_pI2CMaster->Read (m_ucSlaveAddress, &ucStatus, sizeof ucStatus) == sizeof ucStatus
		    && (ucStatus & DS3231_BIT_OSF))
		{
			ucStatus &= ~DS3231_BIT_OSF;
			u8 ClearOSF[2] = {DS3231_REG_STATUS, ucStatus};
			m_pI2CMaster->Write (m_ucSlaveAddress, ClearOSF, sizeof ClearOSF);
		}
	}

	return TRUE;
}
