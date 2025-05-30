/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * display/ST7306.h
 *
 * Driver for the Sitronix ST7306 TFT driver
 */

#pragma once

#include <kernel/kernel.h>

#include <base/Span.h>

class ST7306
{
protected:
    #pragma region Hardware interface, to be implemented by subclass

    //! Writes a command byte to the chip
    virtual void WriteCommand(uint8_t cmd) = 0;
    //! Writes data to the chip
    virtual void WriteData(Span data) = 0;
    //! Reads data from the chip
    virtual void ReadData(Buffer data) = 0;

    #pragma endregion

    #pragma region Configuration helpers for Command Table 1

    //! Triggers a software reset of the display controller
    /*!
     * It is necessary to wait 5 ms before sending any new command after
     * triggering reset. It is also necessary to wait at least 120 ms
     * before waking up the display if reset was sent in sleep mode.
     */
    void Reset() { Configure(Command::Reset); }
    //! Wakes up or puts the display to sleep
    /*!
     * It is necessary to wait 5 ms before sending any new command after going
     * to or waking from sleep. It is also necessary to wait at least 100 ms
     * after waking up before entering sleep again.
     */
    void Sleep(bool enable = true) { Configure(enable ? Command::SleepIn : Command::SleepOut); }
    //! Enables or disables the partial mode (240 lines max instead of 480)
    void Partial(bool enable = true) { Configure(enable ? Command::PartialOn : Command::PartialOff); }
    //! Enables or disables the inverted display mode
    void Invert(bool enable = true) { Configure(enable ? Command::InvertOn : Command::InvertOff); }
    //! Enables or disables output to the display
    void EnableDisplay(bool enable = true) { Configure(enable ? Command::DisplayOn : Command::DisplayOff); }

    //! Start and end column address (inclusive) of the 24-bit 12x2 cell
    struct ColumnAddress { int xs, xe; };
    void Configure(ColumnAddress p) { Configure2(Command::SetColumnAddress, mask(p.xs, 6) | mask(p.xe, 6, 8)); }
    //! Start and end row address (inclusive) of the 24-bit 12x2 cell
    struct RowAddress { int ys, ye; };
    void Configure(RowAddress p) { Configure2(Command::SetRowAddress, mask(p.ys, 8) | mask(p.ye, 8, 8)); }

    //! Starts memory write operation - all data sent after this command will be written to the configured memory window, starting at the beginning
    void WriteBegin() { Configure(Command::Write); }
    //! Continues memory write operation - all data sent after this command will be written to the configured memory window, continuing where left off
    void WriteContinue() { Configure(Command::WriteContinue); }

    //! Configures the TE output line
    struct TearOutput
    {
        //! Enable the TE output
        bool enable;
        //! Enable TE output high after every scan line
        bool hsync;
        //! Configure TE output to the specified scan line
        unsigned line;
    };
    void Configure(TearOutput p)
    {
        ASSERT(!(p.hsync && p.line));
        if (!p.enable) Configure(Command::TearOff);
        else if (p.line) Configure2(Command::TearScanline, TO_BE16(mask(p.line, 8)));
        else Configure1(Command::TearOn, mask(p.hsync, 1));
    }

    //! Configures the order of memory access
    struct MemoryAccess
    {
        //! Lines are refreshed from bottom to top if true
        bool gateScanOrder;
        //! Bit order is flipped (LSB to MSB) if true
        bool dataOrder;
        //! Data is updated in columns instead of rows if true
        bool mv;
        //! Columns are written right to left if true
        bool mx;
        //! Rows are written bottom to top if true
        bool my;
    };
    void Configure(MemoryAccess p) { Configure1(Command::MemoryAccessControl,
        mask(p.gateScanOrder, 1, 2) | mask(p.dataOrder, 1, 3) |
        mask(p.mv, 1, 5) | mask(p.mx, 1, 6) | mask(p.my, 1, 7)); }

    //! Configures the offset of the first vertical line to be output to display
    void VerticalOffset(unsigned offset) { Configure2(Command::VerticalScrollStart, TO_BE16(offset)); }
    //! Enables or disables the Low Power Mode (up to 8 Hz refresh)
    void LowPowerMode(bool enable = true) { Configure(enable ? Command::LowPowerMode : Command::HighPowerMode); }
    //! Configures the data format for display output
    struct DataFormat
    {
        //! 8-color mode (pixels grouped by 3 instead of by 4)
        bool color8;
        //! Data is read in opposite direction if true
        bool xde;
        //! Use BGR instead of RGB order if true
        bool bgr;
        //! Use packed format (3 instead of 4 bytes per pixel)
        bool packed;
    };
    void Configure(DataFormat p) { Configure1(Command::DataFormat,
        mask(p.color8, 1, 5) | mask(p.xde, 1, 4) | mask(p.bgr, 1, 1) | mask(p.packed, 1)); }

    #pragma endregion

    #pragma region Configuration helpers for Command Table 2

    //! Configure Gate (line) timing parameters
    struct GateTiming
    {
        //! Enable the compensation period in Low Power Mode
        bool compensation;
        //! Compensation mode - true = AC, false = DC
        bool ac;
        //! Compensation frame interval - 0-15
        int interval;
        //! Compensation clock divider (1,2,4,8)
        int clkdiv;
        //! First compensation pulse before data upload end (0-15)
        int fcmp;
        //! Compensation pulse width (0-255)
        int cmpw;
    };
    void Configure(GateTiming p) { Configure3(Command::GateTimingControl,
        mask(p.compensation, 1, 5) | mask(p.ac, 1, 4) | mask(p.interval, 4) |
        mask(__builtin_ctz(p.clkdiv), 2, 12) | mask(p.fcmp, 4, 8) |
        mask(p.cmpw, 8, 16)); }

    //! Configure the number of lines of the display
    struct GateLines { unsigned lines; };
    void Configure(GateLines p) { Configure1(Command::GateSetting, mask(p.lines >> 2, 7)); }

    //! Configure the first output gate offset (e.g. offset 7 means gate 8 outputs RAM address 0)
    void FirstGate(unsigned offset) { Configure2(Command::FirstGateSetting, TO_BE16(mask(offset, 9))); }

    //! Configure the frame rate for HPM (16/32) and LPM (0.25-8 in powers of 2) respectively
    struct FrameRate { float hfra, lfra; };
    void Configure(FrameRate params) { Configure1(Command::FrameRateControl,
        mask(__builtin_ctz(params.hfra / 16), 1, 4) |
        mask(__builtin_ctz(params.lfra * 4), 3)); }

    //! Default update period waveform
    static constexpr uint64_t DefaultEq = 0x0546777777777645;
    //! Configure gate update waveform in High Power Mode
    /*!
     * Every nibble represents the state during one of 16 periods
     * 0 - floating, 1 - VGL, 4 - GND, 5 - NAVDD, 6 - AVDD, 7 - VGH
     */
    struct GateEqHPM { bool enable; uint64_t eq; };
    void Configure(GateEqHPM p) { Configure2(Command::GateEqHPM, 0xF6A5 | mask(p.enable, 1, 6)); WriteEq(p.eq); }
    //! Configure gate update waveform in Low Power Mode
    /*!
     * Every nibble represents the state during one of 16 periods
     * 0 - floating, 1 - VGL, 4 - GND, 5 - NAVDD, 6 - AVDD, 7 - VGH
     */
    struct GateEqLPM { uint64_t eq; };
    void Configure(GateEqLPM p) { Configure(Command::GateEqLPM); WriteEq(p.eq); }

    //! Enable source waveform control
    struct SourceEq { bool enable; };
    void Configure(SourceEq p) { Configure1(Command::SourceEq, 0x3 | mask(p.enable, 1, 4)); }

    //! Configure panel layout
    struct Panel {
        //! dot inversion (0 = column inversion, 1/2 = n-dot inversion)
        uint8_t dotinv;
        //! gate scan mode (0 = frame interval, 1/2 = n-line interval)
        uint8_t scan;
        //! interlace layout (0 = no interlace, 1/2 = n-line interlace)
        uint8_t interlace;
    };
    void Configure(Panel p) { Configure1(Command::PanelSetting,
        mask(p.dotinv, 2, 5) | mask(2 - p.scan, 2, 2) | mask(2 - p.interlace, 2)); }

    //! Gamma mode setting
    struct Gamma { bool mono; };
    void Configure(Gamma p) { Configure1(Command::GammaSetting, mask(p.mono, 1, 5)); }

    //! Enable Clear RAM
    void ClearRam(bool enable) { Configure1(Command::ClearRam, mask(enable, 1, 7) | 0x4F); }

    //! Configure Gate Voltages in 0.5V steps
    struct GateVoltage
    {
        // VGL, range -15V to -6V
        float low;
        // VGH, range 8 to 16.5V
        float high;
    };
    void Configure(GateVoltage p) { Configure2(Command::GateVControl,
        mask(p.high * 2 - 16 + 0.5f, 5) |
        mask(-p.low * 2 - 12 + 2 + 0.5f, 5, 8)); }

    //! Configure High Positive Voltages for up to four modes (3.7V to 6V in 0.02V steps)
    struct SourceVoltageHP { float v1, v2 = NAN, v3 = NAN, v4 = NAN; static constexpr float base = 3.7, mul = 50; };
    void Configure(SourceVoltageHP p) { ConfigureVoltage(Command::HighPVControl, p); }
    //! Configure Low Positive Voltages for up to four modes (0V to 2V in 0.02V steps)
    struct SourceVoltageLP { float v1, v2 = NAN, v3 = NAN, v4 = NAN; static constexpr float base = 0, mul = 50; };
    void Configure(SourceVoltageLP p) { ConfigureVoltage(Command::LowPVControl, p); }
    //! Configure High Negative Voltages for up to four modes (-5V to -2.5V in 0.02V steps)
    struct SourceVoltageHN { float v1, v2 = NAN, v3 = NAN, v4 = NAN; static constexpr float base = -2.5, mul = -50; };
    void Configure(SourceVoltageHN p) { ConfigureVoltage(Command::HighNVControl, p); }
    //! Configure Low Negative Voltages for up to four modes (-1.8V to +1V in 0.02V steps)
    struct SourceVoltageLN { float v1, v2 = NAN, v3 = NAN, v4 = NAN; static constexpr float base = 1, mul = -50; };
    void Configure(SourceVoltageLN p) { ConfigureVoltage(Command::LowNVControl, p); }

    //! Configure Source Gamma Voltages
    struct SourceGammaVoltage {
        //! Positive Gamma Voltage 1 (2.2V to 5V in 0.02V steps)
        float vp1;
        //! Positive Gamma Voltage 2 (1.2V to 4V in 0.02V steps)
        float vp2;
        //! Negative Gamma Voltage 1 (-3.8V to -1V in 0.02V steps)
        float vn1;
        //! Negative Gamma Voltage 2 (-2.8V to 0V in 0.02V steps)
        float vn2;
    };
    void Configure(SourceGammaVoltage p) { Configure4(Command::GammaVControl,
        mask((p.vp1 - 2.2f) * 50 + 0.5f, 8) |
        mask((p.vp2 - 1.2f) * 50 + 0.5f, 8, 8) |
        mask((p.vn1 + 1) * -50 + 0.5f, 8, 16) |
        mask((p.vn2 + 0) * -50 + 0.5f, 8, 24)); }

    //! Select pre-configure source voltage group (1-4)
    void SourceVoltageMode(unsigned group) { Configure1(Command::SourceVSel, mask(group - 1, 2)); }
    //! Configure auto power down
    void AutoPowerDown(bool enable = true) { Configure1(Command::AutoPower, 0x7F | mask(enable, 1, 7)); }
    //! Booster enable/disable
    void EnableBooster(bool enable = true) { Configure1(Command::BoosterEnable, mask(enable, 1)); }

    //! Configure NVM Load options
    struct NVLoadControl
    {
        //! Load ID1/2/3
        bool id;
        //! Load Source Voltages
        bool vs;
        //! Load NVM values when waking from sleep
        bool sleepOut;
        //! Load NVM values on timer
        bool timer;
    };
    void Configure(NVLoadControl p) { Configure2(Command::NVLoadControl, 0x17 |
        mask(p.id, 1, 1) | mask(p.vs, 1, 2) | mask(p.sleepOut, 1, 9) | mask(p.timer, 1, 10)); }

    //! Oscillator enable/disable
    void EnableOscillator(bool enable = true) { Configure2(Command::OscSetting, 0xE926 | mask(enable, 1, 7)); }
    //! EXTB pin control
    void ExtB(bool level) { Configure1(Command::ExtBControl, level ? 0x5A : 0xA5); }

    #pragma endregion

    #pragma region ID and status access

    union DisplayStatus
    {
        uint32_t value;
        struct
        {
            bool : 1;
            bool dataOrder : 1;
            bool gateScanOrder : 1;
            bool : 1;
            bool mv : 1;
            bool mx : 1;
            bool my : 1;
            bool booster : 1;

            bool normal : 1;
            bool sleepOut : 1;
            bool partial : 1;
            bool lpm : 1;
            bool packed : 1;
            bool bgr : 1;
            bool xde : 1;
            bool color8 : 1;

            bool : 1;
            bool te : 1;
            bool on : 1;
            bool : 2;
            bool inv : 1;
            bool : 1;
            bool vs : 1;
        };
    };

    uint32_t ReadID() { return Read(uint8_t(Command::ReadID) | (3 << 8)); }
    DisplayStatus ReadStatus() { return { .value = Read(uint8_t(Command::ReadStatus) | (3 << 8)) }; }

    #pragma endregion

    #pragma region Helpers for subclasses

    //! Converts data in standard planar 1bpp format to ST7306 cell format
    /*!
     * @param xCells number of horizonal cells to convert (1 cell = 4 pixels wide)
     * @param yCells number of vertical cells to convert (1 cell = 2 pixels high)
     * @param fbStride specifies the stride to the next planar row in fb
     * @param outStride specifies the stride to the next vertically aligned output cell, i.e. two rows
     */
    static uint8_t* FormatConvert(const uint8_t* fb, uint8_t* out, unsigned xCells, unsigned yCells, unsigned fbStride, unsigned outStride);

    #pragma endregion

protected:
    #pragma region Configuration helpers

    enum struct Command : uint8_t
    {
        // Command table 1
        Nop = 0x00,
        Reset = 0x01,
        ReadID = 0x04,
        ReadStatus = 0x09,
        ReadPowerMode = 0x0A,
        SleepIn = 0x10,
        SleepOut = 0x11,
        PartialOn = 0x12,
        PartialOff = 0x13,
        InvertOff = 0x20,
        InvertOn = 0x21,
        DisplayOff = 0x28,
        DisplayOn = 0x29,
        SetColumnAddress = 0x2A,
        SetRowAddress = 0x2B,
        Write = 0x2C,
        TearOff = 0x34,
        TearOn = 0x35,
        MemoryAccessControl = 0x36,
        VerticalScrollStart = 0x37,
        HighPowerMode = 0x38,
        LowPowerMode = 0x39,
        DataFormat = 0x3A,
        WriteContinue = 0x3C,
        TearScanline = 0x44,
        ReadID1 = 0xDA,
        ReadID2 = 0xDB,
        ReadID3 = 0xDC,

        // Command table 2
        GateTimingControl = 0x62,
        GateSetting = 0xB0,
        FirstGateSetting = 0xB1,
        FrameRateControl = 0xB2,
        GateEqHPM = 0xB3,
        GateEqLPM = 0xB4,
        SourceEq = 0xB7,
        PanelSetting = 0xB8,
        GammaSetting = 0xB9,
        ClearRam = 0xBB,
        GateVControl = 0xC0,
        HighPVControl = 0xC1,
        LowPVControl = 0xC2,
        HighNVControl = 0xC4,
        LowNVControl = 0xC5,
        GammaVControl = 0xC8,
        SourceVSel = 0xC9,
        SetID1 = 0xCA,
        SetID2 = 0xCB,
        SetID3 = 0xCC,
        AutoPower = 0xD0,
        BoosterEnable = 0xD1,
        NVLoadControl = 0xD6,
        OscSetting = 0xD8,
        ReadOTP = 0xE9,
        ExtBControl = 0xEC,
        NVMControl1 = 0xF8,
        NVMControl2 = 0xFA,
        NVMReadEnable = 0xFB,
        NVMProgramEnable = 0xFC,
    };

    void Configure(Command cmd) { WriteCommand(uint8_t(cmd)); }
    void Configure(Command cmd, Span data) { Configure(uint8_t(cmd) | (data.Length() << 8), data.Element<uint32_t>()); }
    void Configure(uint32_t cmdAndLength, uint32_t data);
    void Configure(uint32_t cmdAndLength, const char* data);
    void WriteEq(uint64_t data);

    template<typename T> void Configure1(Command cmd, T data) { Configure(uint8_t(cmd) | (1 << 8), unsafe_cast<uint8_t>(data)); }
    template<typename T> void Configure2(Command cmd, T data) { Configure(uint8_t(cmd) | (2 << 8), unsafe_cast<uint16_t>(data)); }
    template<typename T> void Configure3(Command cmd, T data) { Configure(uint8_t(cmd) | (3 << 8), unsafe_cast<uint32_t>(data)); }
    template<typename T> void Configure4(Command cmd, T data) { Configure(uint8_t(cmd) | (4 << 8), unsafe_cast<uint32_t>(data)); }
    template<typename T> void ConfigureVoltage(Command cmd, T data)
    {
        if (isnanf(data.v2)) { data.v2 = data.v1; }
        if (isnanf(data.v3)) { data.v3 = data.v2; }
        if (isnanf(data.v4)) { data.v4 = data.v3; }
        Configure4(cmd, ConvertVoltage(data, data.v1) | ConvertVoltage(data, data.v2) << 8 | ConvertVoltage(data, data.v3) << 16 | ConvertVoltage(data, data.v4) << 24);
    }
    template<typename T> uint8_t ConvertVoltage(T data, float value)
    {
        return mask((value - T::base) * T::mul + 0.5f, 8);
    }

    constexpr unsigned mask(unsigned value, unsigned bits, unsigned offset = 0)
    {
        ASSERT(!(value & ~MASK(bits)));
        return (value & MASK(bits)) << offset;
    }

    #pragma endregion

    #pragma region Readout helpers

    uint32_t Read(uint32_t cmdAndLength);

    #pragma endregion
};
