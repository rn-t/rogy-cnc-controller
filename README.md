# Rogy-CNC-Controller

## PCB

## Firmware

### キーマップの書き換え

キーマップを書き換えるには`firmware/Core/Inc/settings.h`の該当する`keycodes`と`modifiers`の要素を書き換えます．

注) `keycode`は`ASCII`などではありません．[ここ](http://www2d.biglobe.ne.jp/~msyk/keyboard/layout/usbkeycode.html)などを参照して`keycode`を調べてください．

### LED

LEDの色を変更するには`firmware/Core/Inc/settings.h`の該当する`LEDsettings`の値を変更します。値はRGBの順で並んでいます。