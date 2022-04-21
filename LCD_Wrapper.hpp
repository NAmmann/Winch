#ifndef LCD_WRAPPER_H
#define LCD_WRAPPER_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define N_ROWS 4
#define N_COLS 20

class LCD_Wrapper : public Print
{
    private:
        LiquidCrystal_I2C _display;

        char oldData[N_COLS * N_ROWS + 1];
        char newData[N_COLS * N_ROWS + 1];
        unsigned int idx;
        unsigned int updateIdx;

    public:
        LCD_Wrapper(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows)
            : _display(lcd_Addr, lcd_cols, lcd_rows)
            , idx(0)
            , updateIdx(0)
        {
            for (unsigned int i = 0; i < N_ROWS * N_COLS; ++i) {
                this->oldData[i] = 0;
                this->newData[i] = ' ';
            }
            this->oldData[N_ROWS * N_COLS] = '\0';
            this->oldData[N_ROWS * N_COLS] = '\0';
        }

        void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS ) { this->_display.begin(cols, rows, charsize); }
        void home() { this->idx = 0; }
        void init() { this->_display.init(); }
        void noDisplay() { this->_display.noDisplay(); }
        void display() { this->_display.display(); }
        void noBlink() { this->_display.noBlink(); }
        void blink() { this->_display.blink(); }
        void noCursor() { this->_display.noCursor(); }
        void cursor() { this->_display.cursor(); }
        void noBacklight() { this->_display.noBacklight(); }
        void backlight() { this->_display.backlight(); }
        void createChar(uint8_t c, uint8_t pixels[]) { this->_display.createChar(c, pixels); }
        void setCursor(uint8_t col, uint8_t row)
        {
            this->idx = col + N_COLS * row;
        }

        void updateDisplay(unsigned int n = N_COLS * N_ROWS)
        {
            unsigned int written = 0;
            for (unsigned int i = 0; i < N_COLS * N_ROWS; ++i) {
                if (newData[updateIdx] != oldData[updateIdx]) {
                    this->_display.setCursor(updateIdx % N_COLS, updateIdx / N_COLS);
                    this->_display.write(newData[updateIdx]);
                    oldData[updateIdx] = newData[updateIdx];
                    if (++written == n) return;
                }
                if (++updateIdx == N_COLS * N_ROWS) updateIdx = 0;
            }
        }

        void clear()
        {
            for (unsigned int i = 0; i < N_ROWS * N_COLS; ++i) {
                this->newData[i] = ' ';
            }
            idx = 0;
            updateIdx = 0;
        }

#if defined(ARDUINO) && ARDUINO >= 100
        virtual size_t write(uint8_t d)
        {
            this->newData[idx++] = d;
            if (idx == N_COLS * N_ROWS) idx = 0;
            
            return 1;
        }

        virtual size_t write(const uint8_t *buffer, size_t size)
        {
            for (unsigned int i = 0; i < size; ++i) {
                this->newData[idx++] = buffer[i];
                if (idx == N_COLS * N_ROWS) idx = 0;
            }
            return size;
        }
#else
        virtual void write(uint8_t d)
        {
            this->newData[idx++] = d;
            if (idx == N_COLS * N_ROWS) idx = 0;
        }

        virtual void write(const uint8_t *buffer, size_t size)
        {
            for (unsigned int i = 0; i < size; ++i) {
                this->newData[idx++] = buffer[i];
                if (idx == N_COLS * N_ROWS) idx = 0;
            }
        }
#endif
};
#endif
