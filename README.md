# GPIB-Listener-USB-Logger  
# DMMのGPIB出力をUSBシリアルに送ってロギング  
  
・アドバンテストのデジタルマルチメーターTR6846(GPIBインターフェース付き)を入手したので、計測データをPCでロギングできるようにしてみました。  
  
![system_diagram](https://github.com/user-attachments/assets/c61e526c-b03f-4a65-a4de-52e0ac8bca7f)

  
**●ファイル**  
■GPIBlistenerUSB：トークオンリーに設定したDMMから受け取った1行分の文字列を一定インターバル(今の設定は0.2秒)でUSBシリアルに送信するユニットのArduinoソースです。  
■3Ddata：上記ユニットの3Dデータです。stl以外にAutodeskFusionのファイルもアップしたので適宜変更してください。  
■DMMlogger.html：PC側で受け取ったデータを一定インターバルでロギングしcsvファイルを作成するアプリです。  
  
**●GPIBシリアルユニット**  
・ハードウェアとしてはArduinoNANOとGPIB用コネクタを直結しただけの物です。  
・ソフトウェアは当初こちらを参考に結線してコードを焼いてみたのですが、私の用途ではPC側からの機器の制御は不要なので、リッスンオンリーとしたコードを作って焼き直しました。そのため未使用の配線も残っていますが未結線にしておけばOKです。  
>gpib-conv-arduino-nano  
https://github.com/JacekGreniger/gpib-conv-arduino-nano  

・参考までにGPIBコネクタのピン配です。  
![GPIBpin](https://github.com/user-attachments/assets/c07e5797-d3c2-43d8-928c-c24af66da4ae)

・GPIBコネクタとArduinoNANOの結線はソースに記述してあります。(記述してませんがGNDも忘れずに)  
  

![GPIBunit](https://github.com/user-attachments/assets/732c0782-f8b0-4086-b972-6f68c2644444)


・GPIB用コネクタは、手持ちのジャンクからセントロニクスプリンタ用36pinフラットケーブルタイプをばらして24pinを作成しました。  
・Arduino NANOも手持ちの古いmini-Bタイプを使いました。もちろんType-Cの物でもOKでしょうが、ケースの穴位置の多少の修正が必要かもしれません。  
・このユニットの機能としては、DMMから受け取った最新の文字列を200ms間隔でそのままPCにシリアル出力するだけです。DMM側のサンプルレートによってデータの間隔が異なるためにこのような仕様としました。ソースのTXINTERVALの値でインターバルを変えたり受け取ったデータを毎回送るようにすることもできます。  
・参考にしたコードでは論理Hの出力時にロジックHの電圧を印可するようになってますが、私のコードでは論理Hはハイインピーダンスとしています。GPIBの規格としてはこの方が正しいと思いますが、もし不安定なら外付けプルアップ抵抗を追加するか、ATmegaの内部プルアップを使えばいいと思います(後者の場合PORTの操作を追加する必要があります。)  
・DMM側の設定でトークオンリーモードで行末はLF(0x0A)とするようにしておく必要があります。TR684xシリーズではデータの頭に3文字分の測定モードを示すヘッダを付けることができますが、ヘッダがあるとロガー側アプリでその旨対応しているのでお勧めです。  
![GPIBsetting](https://github.com/user-attachments/assets/17605abb-a7fc-4087-8a0c-9e1f38a2648f)

  
**●PC側ロガーアプリ**  
・ブラウザで実行するアプリです。サーバーではなくローカルPC上で動くのですが、フォントやアイコンやチャート機能の読み込みなどがあるので実行にはネット接続が必要です。  
・次に示すのはリチウムイオン電池を3.7Ωの抵抗で放電した時の端子電圧の推移です。  
![Li Disccharge3](https://github.com/user-attachments/assets/0a7a33dc-4ccc-4745-b544-76d1073ebc3a)

・次に示すのは3Dプリンタのベッド加熱時の温度の推移です。
![BedHeat60deg](https://github.com/user-attachments/assets/9925f59b-d6a1-49f7-b655-861c6443d379)

  
・使い方は特に説明は不要かと思います。アドバンテストのTR684xシリーズ以外で使うにはヘッダー文字の解釈部分の変更が必要かもしれません。コード自体はテキストファイルで、かつビルドの必要もないので修正は簡単だと思います。ChatGPTに丸ごとアップしてから「〇〇を××にしたい」と打てば変更箇所を教えてくれます。(このアプリもそうやって開発しました)   
  
**●TR6846**  
・DMM本体についてはこちら↓にアップしました。  
https://minkara.carview.co.jp/userid/3336538/blog/48369326/

以上です。
