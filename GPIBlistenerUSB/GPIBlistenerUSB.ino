/*****************************************************************************
*
*	GPIBlistenerUSB.ino -- GPIB受信文字列をArduino NANOでUSBにシリアル送信
*
*	GPIBから受け取ったデータを一定インターバルでシリアル送信する
*
*	GPIB機器側はトークオンリーモードに設定しておく
+	GPIB機器側の行末はCRLFとしておく
*	このユニットはリッスンオンリーなのでGPIB機器への送信は行わない
*
*	rev1.0	2025/04/10	initial revision by	Toshi
*
*****************************************************************************/
#include <MsTimer2.h>
#include "string.h"

#define TXINTERVAL 200	// シリアル送信インターバル[ms] 0なら毎回送信

// Arduino NANO ピン設定
#define DIO1 A0	// GPIB 1	(ATmega:PC0)
#define DIO2 A1	// GPIB 2	(ATmega:PC1)
#define DIO3 A2	// GPIB 3	(ATmega:PC2)
#define DIO4 A3	// GPIB 4	(ATmega:PC3)
#define DAV 3	// GPIB 6	(ATmega:PD3)
#define NRFD 4	// GPIB 7	(ATmega:PD4)
#define NDAC 5	// GPIB 8	(ATmega:PD5)
#define DIO5 A4	// GPIB 13	(ATmega:PC4)
#define DIO6 A5	// GPIB 14	(ATmega:PC5)
#define DIO7 8	// GPIB 15	(ATmega:PB0)
#define DIO8 9	// GPIB 16	(ATmega:PB1)
#define PULSE 12
#define LED 13

#define BITDAV (1<<3)	// b3
#define BITNRFD (1<<4)	// b4
#define BITNDAC (1<<5)	// b5
// オープンコレクタ駆動とするので論理Hでハイインピーダンス、論理Lで0出力
#define SetNRFD(x) ( DDRD = (x)? (DDRD & ~BITNRFD) : (DDRD | BITNRFD) )
#define SetNDAC(x) ( DDRD = (x)? (DDRD & ~BITNDAC) : (DDRD | BITNDAC) )
#define ReadDAV() (PIND & BITDAV)
// データ読み込みLSB6bit + MSB2bit GPIBは負論理なのでビット反転
#define ReadGPIB() (~((PINC & 0x3f) | (PINB & 0x03) << 6))

#define CR 0x0D
#define LF 0x0A

// デバッグ用
inline void pulseOn() { digitalWrite(PULSE, HIGH); }
inline void pulseOff() { digitalWrite(PULSE, LOW); }
inline void ledOn() { digitalWrite(LED, HIGH); }
inline void ledOff() { digitalWrite(LED, LOW); }

// グローバル変数
#define GPIB_BUF_SIZE 64
char GPIBrxbuf[GPIB_BUF_SIZE];
char GPIBrxstr[GPIB_BUF_SIZE];
volatile bool fRxOk;

// プロトタイプ宣言
void TimerProc();
void GPIBrx();

/*----------------------------------------------------------------------------
	セットアップ
----------------------------------------------------------------------------*/
void setup()
{
	// ピンモードの設定
	pinMode(PULSE, OUTPUT);		// デバッグ用
	pinMode(LED, OUTPUT);

	PORTD = 0;	// 制御線は出力時に0
	SetNRFD(0);	// 0出力
	SetNDAC(0);

	Serial.begin(115200);	// ハードウェアシリアル

#if TXINTERVAL	// インターバル送信なら
	// タイマー割込み設定
	MsTimer2::set(TXINTERVAL, TimerProc);
	MsTimer2::start();	// タイマー割込み開始
#endif //TXINTERVAL
}

/*----------------------------------------------------------------------------
	メインループ
----------------------------------------------------------------------------*/
void loop()
{
	GPIBrx();	// GPIBから受信

#if !TXINTERVAL	// インターバル送信でないなら
	Serial.print(GPIBrxstr);	// USBシリアルに送信
#endif //!TXINTERVAL

	digitalWrite(LED, !digitalRead(LED));	// 受信ごとにLED反転
}
/*----------------------------------------------------------------------------
	タイマー割込み
----------------------------------------------------------------------------*/
void TimerProc()
{
	static bool fin;

	if (fin)	// 現在実行中だったら
	{
		return;	// 何もしない
	}
	fin = true; // 実行中フラグセット
	sei();		// 他の割り込みを許可

	if (fRxOk)	// GPIBから受信していたら
	{
		Serial.print(GPIBrxstr);	// USBシリアルに送信
	}
	fin = false;	// 実行中フラグをクリア
}
/*----------------------------------------------------------------------------
	GPIBから受信
----------------------------------------------------------------------------*/
#define TIMEOUT 2000	// タイムアウト時間[ms]
void GPIBrx()
{
	uint8_t c, index;
	uint32_t t;

pulseOff();
	index = 0;
	do {
		SetNRFD(1);		// NRFDをHに
		t = millis();
		while (ReadDAV())	// DAVがLになるのを待つ
		{
			if (millis() - t > TIMEOUT)	// タイムアウトなら
			{
				SetNRFD(0);		// NRFDをLに
				fRxOk = false;	// 受信なし
				return;
			}
		}
		SetNRFD(0);	// NRFDをLに

		c = ReadGPIB();			// データ読み込み
		GPIBrxbuf[index++] = c;	// バッファに格納
		if (index >= GPIB_BUF_SIZE - 1)	// バッファフル？(行末の'\0'用に-1)
		{
			fRxOk = false;		// 受信なし
			return;
		}
		SetNDAC(1);	// NDACをHに
		t = millis();
		while (!ReadDAV())	// DAVがHになるのを待つ
		{
			if (millis() - t > TIMEOUT)	// タイムアウトなら
			{
				SetNDAC(0);		// NDACをLに
				fRxOk = false;	// 受信なし
				return;
			}
		}
		SetNDAC(0);	// NDACをLに
	} while (c != LF);			// 行末のLFを受けるまでループ
	GPIBrxbuf[index] = '\0';	// データを0でターミネートして文字列に
	cli();	// 割り込み禁止
	strcpy(GPIBrxstr, GPIBrxbuf);// シリアル送信用バッファにコピー
	fRxOk = true;	// 受信完了
	sei();	// 割り込み再開
pulseOn();
}
/*** end of "GPIBlistenerUSB.ino" ***/
