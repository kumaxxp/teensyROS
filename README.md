# teensyROS

teensy4.0にBNO055を使って9defを行う。
micro-ROSを使ってPCにデータを送り、Rvizなどで表示します。（作業中）

+ teensyros.ino ...teensy4.0側に焼き込むファイル
+ read_all_data ...BNO055からデータを全部取り出すファイルのサンプル。

> Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  //変更前
> Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  //変更後

アドレスが1つずれていたので補正。
もしかしたら、どこかのピンをGNDに落とす必要があるのかも。


