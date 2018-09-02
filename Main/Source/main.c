/*
 * pencil_prod_counter
 * ペンシルラインケースカウンター
 * Ａ，Ｂラインのケースをカウントし、レシーバーへ送信する
 *
 * カウント時に送信
 * "Q"を受信したときは現在値を送信
 * "R"を受信したときはカウンタをリセットして送信
 *
 * 送信フォーマット
 * C:<カウンタ１(8ﾊﾞｲﾄの16進文字)>:<カウンタ2(8ﾊﾞｲﾄの16進文字)>:<リセットからの経過時間秒(8ﾊﾞｲﾄの16進文字)>
 */

//#define	_DEBUG
#define _DUMMY_MODE // MONOSTICKで仮想動作

#include <AppHardwareApi.h>
#include "utils.h"
#include "ToCoNet.h"
#include "serial.h"         // シリアル用
#include "string.h"
#include "sprintf.h"
#include "ToCoNet_mod_prototype.h" // ToCoNet モジュール定義(無線で使う)

// ToCoNet 用パラメータ
#define APP_ID   0x67720103
#define CHANNEL  18

#define UART_BAUD 115200 	// シリアルのボーレート

#ifndef _DUMMY_MODE
#define LED  9  // デジタル出力 4
#else
#define LED  16 // モノスティックのLEDは16
#endif
#define DI1  12 // デジタル入力 1
#define DI2  13 // デジタル入力 2

#define DI1_MASK    (1UL << DI1)
#define DI2_MASK    (1UL << DI2)

#ifdef _DEBUG
static tsFILE sSerStream;          // シリアル用ストリーム
static tsSerialPortSetup sSerPort; // シリアルポートデスクリプタ
#endif

static uint32 u32Seq = 0;          // 送信パケットのシーケンス番号
static uint32 uCounter1 = 0;
static uint32 uCounter2 = 0;
static bool bSendData = TRUE;
static uint32 uTimePassed = 0;

// デバッグメッセージ出力用
#ifdef _DEBUG
#define debug(...) vfPrintf(&sSerStream, LB __VA_ARGS__)
#else
#define debug(...)
#endif

static bool_t sendBroadcast();

#ifdef _DEBUG
// デバッグ出力用に UART を初期化
static void vSerialInit() {
    static uint8 au8SerialTxBuffer[96];
    static uint8 au8SerialRxBuffer[32];

    sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
    sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
    sSerPort.u32BaudRate = UART_BAUD;
    sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
    sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
    sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
    sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
    sSerPort.u8SerialPort = E_AHI_UART_0;
    sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
    SERIAL_vInit(&sSerPort);

    sSerStream.bPutChar = SERIAL_bTxChar;
    sSerStream.u8Device = E_AHI_UART_0;
}
#endif

// ユーザ定義のイベントハンドラ
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{
#ifdef _DUMMY_MODE
    static uint32 io = 0;
#endif
    static int sw = 0;
    static int wt = 0;

    // 起動時
    if (eEvent == E_EVENT_START_UP) {
        sendBroadcast();
    }
#ifdef _DUMMY_MODE
    // 1秒 周期のシステムタイマ通知
    else if (eEvent == E_EVENT_TICK_SECOND) {
        io += (1UL << (DI1 - 1));//ダミーモードの場合はテキトーにON/OFF

        uTimePassed++;
    }
#endif
    // 4ms 周期のシステムタイマ通知
    else if (eEvent == E_EVENT_TICK_TIMER) {
        if(++wt > 40){
            vPortSetHi(LED);    //LED OFF

            wt = 0;
#ifndef _DUMMY_MODE
            uint32 io = u32AHI_DioReadInput();
#endif
            int n = 0;
            if(io & DI1_MASK) n += 1;
            if(io & DI2_MASK) n += 2;

            if((sw&1) != (n&1) && (n&1) == 0) {
                uCounter1++;
                bSendData = TRUE;
            }
            if((sw&2) != (n&2) && (n&2) == 0) {
                uCounter2++;
                bSendData = TRUE;
            }
            sw = n;

            if(bSendData) {
                sendBroadcast();
                bSendData = FALSE;
            }
        }
	}
}

// 電源オンによるスタート
void cbAppColdStart(bool_t bAfterAhiInit)
{
	if (!bAfterAhiInit) {
        // 必要モジュール登録手続き
        ToCoNet_REG_MOD_ALL();
	} else {
        // SPRINTF 初期化
        SPRINTF_vInit128();

		// ユーザ定義のイベントハンドラを登録
		//vAHI_DioInterruptEnable(DI1_MASK | DI2_MASK, 0); // 割り込みの登録

        // LED用
        vPortAsOutput(LED);
        vPortSetHi(LED);

#ifdef _DEBUG
		// デバッグ出力用
		vSerialInit();
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);
#endif

        // ToCoNet パラメータ
        sToCoNet_AppContext.u32AppId = APP_ID;
        sToCoNet_AppContext.u8Channel = CHANNEL;
        sToCoNet_AppContext.bRxOnIdle = TRUE; // アイドル時にも受信
        u32Seq = 0;

        // ユーザ定義のイベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

        // MAC 層開始
        ToCoNet_vMacStart();
    }
}

// スリープからの復帰
void cbAppWarmStart(bool_t bAfterAhiInit)
{
    //今回はスリープしないので無意味
}

// ネットワークイベント発生時
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg)
{
	switch(eEvent) {
	default:
		break;
	}
}

// パケット受信時
void cbToCoNet_vRxEvent(tsRxDataApp *pRx)
{
    static uint32 u32SrcAddrPrev = 0;
    static uint8 u8seqPrev = 0xFF;

    // 前回と同一の送信元＋シーケンス番号のパケットなら受け流す
    if (pRx->u32SrcAddr == u32SrcAddrPrev && pRx->u8Seq == u8seqPrev) {
        return;
    }
    // ペイロードを切り出してデバッグ出力
    char buf[64];
    int len = (pRx->u8Len < sizeof(buf)) ? pRx->u8Len : sizeof(buf)-1;
    memcpy(buf, pRx->auData, len);
    buf[len] = '\0';
#ifdef _DEBUG
    debug("RECV << [%s]", buf);
    debug("current count = %d, %d", uCounter1, uCounter2);
#endif

    if(buf[0] == 'R') {
        vPortSetLo(LED);    //LED ON
        uCounter1 = 0;
        uCounter2 = 0;
        uTimePassed = 0;
        bSendData = TRUE;
    }
    else if(buf[0] == 'Q') {
        vPortSetLo(LED);    //LED ON
        bSendData = TRUE;
    }

    u32SrcAddrPrev = pRx->u32SrcAddr;
    u8seqPrev = pRx->u8Seq;
}

//xを8文字(32bit)の16進数に変換
unsigned char *i2Hex(unsigned char *p, uint32 x)
{
    int i;
    for(i=7;i>=0;i--)
    {
        unsigned char c = (x >> (i << 2)) & 15;
        *p++ = c + ((c <= 9) ? 48 : 55);
    }
    return p;
}

// ブロードキャスト送信を実行
static bool_t sendBroadcast()
{
    vPortSetLo(LED);    //LED ON

    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = ToCoNet_u32GetSerial();//チップのS/N
    tsTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST;

    u32Seq++;
    tsTx.bAckReq = FALSE;
    tsTx.u8Retry = 0x02; // 送信失敗時は 2回再送
    tsTx.u8CbId = u32Seq & 0xFF;
    tsTx.u8Seq = u32Seq & 0xFF;
    tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;

    unsigned char buf[28];
    buf[0] = 'C';
    buf[1] = ':';
    i2Hex(&buf[2], uCounter1);
    buf[10] = ':';
    i2Hex(&buf[11], uCounter2);
    buf[19] = ':';
    i2Hex(&buf[20], uTimePassed);

    memcpy(tsTx.auData, buf, 28);
    tsTx.u8Len = 28;

    // SPRINTF でペイロードを作成
    //SPRINTF_vRewind();
    //vfPrintf(SPRINTF_Stream, "CC%08X:%08X:%08X", u32Seq, uCounter1, uCounter2);
    //memcpy(tsTx.auData, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
    //tsTx.u8Len = SPRINTF_u16Length();

    // 送信
    return ToCoNet_bMacTxReq(&tsTx);
}

// パケット送信完了時
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus)
{
#ifdef _DEBUG
   debug(">> SENT %s seq=%08X", bStatus ? "OK" : "NG", u32Seq);
#endif
}

// ハードウェア割り込み発生後（遅延呼び出し）
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
/*反応悪いのでやめた
    int z=0;
    switch (u32DeviceId) {
    case E_AHI_DEVICE_SYSCTRL: // DIO etc
        if (u32ItemBitmap & DI1_MASK) {
            // interrupt on DI1
            if(bPortRead(DI1)) {
                uCounter1++;
                bSendData = TRUE;
                z=1;
            }
        }
        if (u32ItemBitmap & DI2_MASK) {
            // interrupt on DI2
            if(bPortRead(DI2)) {
                uCounter2++;
                bSendData = TRUE;
                z=2;
            }
        }
        if(z){
            debug("counted");
        }
        break;
    case E_AHI_DEVICE_TICK_TIMER:
    	break;
    default:
    	break;
    }
*/
}

// ハードウェア割り込み発生時
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	return FALSE;
}

// メイン
void cbToCoNet_vMain(void)
{
}
