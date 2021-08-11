;
;	HSP help manager用 HELPソースファイル
;	(先頭が「;」の行はコメントとして処理されます)
;

%type
内蔵命令
%ver
3.6
%note
ver3.6標準命令
%date
2020/06/04
%author
onitama
%url
http://hsp.tv/
%port
Win
Let


%index
button
ボタン表示
%group
オブジェクト制御命令
%prm
goto/gosub "name",*label
"name" : ボタンの名前
*label : 押した時にジャンプするラベル名

%inst
カレントポジションにオブジェクトとして押しボタンを配置します。
ボタンには、"name"で指定された文字列が書かれ、マウスでボタンをクリックすると、*labelで指定した場所にプログラムの制御が移ります。
^
button gotoと記述した場合は、ラベルにジャンプ。button  gosubと記述した場合は、 ラベルにサブルーチンジャンプを行ないます。goto、gosubキーワードを省略した場合には、gotoと同じ動作になります。
^p
例 :
	button gosub "ボタン",*aaa  ; *aaaを呼び出すボタンを作成
^p
^
オブジェクトの大きさはobjsize命令で指定することができます。 ボタンを配置すると、カレントポジションは次の行に自動的に移動します。
^
ボタンが押された時にジャンプして実行されるプログラムでは、システム変数statの初期値としてオブジェクトIDが代入されます。
^
通常は、ボタンの形状としてWindowsシステムで用意されたものが使用されます。
ただし、objimage命令によって画像を使用したカスタムボタンの設定が行なわれている場合は、自由な外観にすることができます。
カスタムボタンの設定については、objimage命令の項を参照してください。


%href
objsize
objimage



%index
chkbox
チェックボックス表示
%group
オブジェクト制御命令
%prm
"strings",p1
"strings" : チェックボックスの内容表示文字列
p1=変数   : チェックボックスの状態を保持する変数

%inst
カレントポジションにオブジェクトとしてチェックボックスを配置します。
チェックボックスには、"strings"で指定した文字列の左側に、カーソルでON/OFFを切り替えることのできるスイッチがついたオブジェクトです。
^
チェックボックスの大きさは、 objsizeで指定されたサイズになり背景はグレーです。
^
p1で指定された数値型変数の内容が0ならばチェックはOFFになり、 1ならばONになります。チェックのON/OFFが変更されると、変数の内容も同時に変化します。

%href
objsize




%index
clrobj
オブジェクトをクリア
%group
オブジェクト制御命令
%prm
p1,p2
p1=0〜(0)   : 消去するオブジェクトID(開始)
p2=-1〜(-1) : 消去するオブジェクトID(終了)( -1の場合は、 最終のIDが指定
されます )

%inst
button命令やmesbox命令などで出したオブジェクトを消去します。
^
p1,p2を省略して、 clrobjだけを実行させると画面上のオブジェクトがすべて消去されます。一部のオブジェクトだけを消去したい場合は、p1に最初のID、p2に最後のIDを指定すればp1〜p2までのオブジェクトだけが消去されます。指定するオブジェクトIDは、 objprm命令などで指定するIDと同じものです。
^
削除されたオブジェクトのIDは、新しくオブジェクトを配置する際には再利用されます。



%index
combox
コンボボックス表示
%group
オブジェクト制御命令
%prm
p1,p2,p3
p1=変数      : コンボボックスの状態を保持する数値型変数
p2=0〜(100)  : 拡張Yサイズ
p3="strings" : コンボボックスの内容を示す文字列

%inst
カレントポジションにオブジェクトとして、コンボボックスを配置します。
コンボボックスは、複数の文字列要素の中から１つを選択させることのできる入力オブジェクトです。
オブジェクトの大きさは、objsize命令で設定した大きさになります。ただし、p2パラメータで選択時のリスト表示のためのYサイズを指定しなければなりません。 (通常は100〜150程度が適当です)
^
「\n」で区切った文字列をp3で指定することで、選択する要素を設定することができます。
たとえば、「APPLE\nORANGE\nGRAPE」という文字列を指定すると、 「APPLE」「ORANGE」「GRAPE」の中から１つを選択するコンボボックスになります。
それぞれの要素には、0から順番にインデックス番号がついています。 前の例では、「APPLE」はインデックス0、「ORANGE」はインデックス1、「GRAPE」はインデックス2というふうに番号がついていきます。
^
この「\n」で区切るというデータ形式は、メモリノートパッド命令で扱う複数行テキストデータと同じです。メモリノートパッド命令で作成したデータをそのままcombox命令にも使用できます。
^
ユーザーが選択をすると、p1で指定した数値型変数にインデックス番号が代入されます。
最初にコンボボックスが配置される際には、p1で設定した変数が示すインデックスが選ばれた状態になります。 (インデックス番号が-1の時は非選択の状態になります)

%href
objsize

%sample
	a=0:objsize 120,24
	combox a,120,"APPLE\nORANGE\nGRAPE"
	stop




%index
input
入力ボックス表示
%group
オブジェクト制御命令
%prm
p1,p2,p3,p4
p1=変数 : 入力のための変数
p2,p3   : メッセージボックスのサイズ（ドット単位）
p4=0〜  : 入力できる最大文字数

%inst
カレントポジションにオブジェクトとして入力ボックスを配置します。 p2,p3で大きさを指定して、場所はカレントポジションからになります。 p2,p3が省略された場合は、objsizeで指定されたサイズになります。
^
入力ボックスは、キーボードから直接入力することのできる小さな窓です。マウスで入力ボックスをクリックしてカーソルを出した状態で、キーボードからパラメータを入力することができるようになります。
入力した値は、p1で指定した変数に代入されます。もし、p1の変数が文字列型だった場合には入力した文字列が、数値型だった場合には入力した値がそのまま変数に反映されます。
^
入力ボックスは初期状態では、p1で指定した変数に代入されていた値が、ボックス内に表示されます。
^
p4で、入力できる最大文字数を指定することができます。p4が省略された場合には、p1で指定された変数に格納できる最大文字数が自動的に割り当てられます。(変数が文字列型以外だった場合はデフォルトで32文字までとなります。)
p4を0に指定した場合は、そのバージョンのWindowsで扱える最大文字数が入力可能になります。
^
入力ボックスを配置すると、カレントポジションは次の行に自動的に移動します。

%href
mesbox
objsize




%index
listbox
リストボックス表示
%group
オブジェクト制御命令
%prm
p1,p2,p3
p1=変数      : リストボックスの状態を保持する数値型変数
p2=0〜(100)  : 拡張Yサイズ
p3="strings" : リストボックスの内容を示す文字列

%inst
カレントポジションにオブジェクトとして、リストボックスを配置します。
リストボックスは、複数の文字列要素の中から１つを選択させることのできる入力オブジェクトです。
オブジェクトの大きさは、objsize命令で設定した大きさになります。ただし、p2パラメータで選択時のリスト表示のためのYサイズを指定しなければなりません。 (通常は100〜150程度が適当です)
^
「\n」で区切った文字列をp3で指定することで、選択する要素を設定することができます。
たとえば、「APPLE\nORANGE\nGRAPE」という文字列を指定すると、 「APPLE」「ORANGE」「GRAPE」の中から１つを選択するリストボックスになります。
それぞれの要素には、0から順番にインデックス番号がついています。 前の例では、「APPLE」はインデックス0、「ORANGE」はインデックス1、「GRAPE」はインデックス2というふうに番号がついていきます。
^
この「\n」で区切るというデータ形式は、メモリノートパッド命令で扱う複数行テキストデータと同じです。メモリノートパッド命令で作成したデータをそのままlistbox命令にも使用できます。
^
ユーザーが選択をすると、p1で指定した数値型変数にインデックス番号が代入されます。
最初にリストボックスが配置される際には、p1で設定した変数が示すインデックスが選ばれた状態になります。 (インデックス番号が-1の時は非選択の状態になります)

%href
objsize

%sample
	a=0:objsize 120,24
	listbox a,120,"APPLE\nORANGE\nGRAPE"
	stop




%index
mesbox
メッセージボックス表示
%group
オブジェクト制御命令
%prm
p1,p2,p3,p4,p5
p1=変数    : 表示メッセージが代入された文字列型変数
p2,p3      : メッセージボックスのサイズ（ドット単位）
p4=0〜(1)  : メッセージボックスのスタイル
p5=0〜(-1) : 入力できる最大文字数

%inst
ウィンドウ上にオブジェクトとして、 メッセージボックス(メッセージ表示用の窓)を配置します。p2,p3で大きさを指定して、場所はカレントポジションからになります。
メッセージボックスのスタイルに設定する値(p4)は、以下の通りになります。
^p
   値 : 対応するキー
 ---------------------------------------------------------------
     0  : スクロール可能なエディットボックス(書き換え不可)
    +1  : スクロール可能なエディットボックス(書き換え可能)
    +4  : 横スクロールバーを付ける
    +8  : 自動ラップ(折り返し)を無効にする
^p
書き換え可能なエディットボックスを作成すると、ユーザーが好きに文字を入力できる簡単なテキストエディタになります。+4(横スクロールバーを付ける)、+8(自動ラップを無効にする)はそれぞれの値をp4に加算することで、複数を同時に指定することができます。
^
p5で、入力できる最大文字数を指定することができます。
p5が0の場合は、そのバージョンのWindowsで扱える最大文字数になります。
p5が省略されるかマイナス値の場合には、p1で指定された変数に格納できる最大文字数が自動的に割り当てられます。

%href
input




%index
objprm
オブジェクトの内容を変更
%group
オブジェクト制御命令
%prm
p1,p2
p1=0〜(0) : オブジェクトID指定
p2        : 変更するパラメータの内容

%inst
button命令やinput,mesbox命令などで画面上に配置したオブジェクトの持つ内容やパラメータを変更します。
^
p1には、オブジェクトIDを指定します。オブジェクトIDは、 0から順番に画面上に表示したオブジェクト１つ１つに割り当てられていく番号です。
オブジェクトIDは、オブジェクトを配置する命令実行後にシステム変数statに代入されています(通常置いた順番に0,1,2…の番号が割り当てられます)。
^
ここで指定されたオブジェクトをp2で指定したパラメータで変更します。
p2で指定するパラメータはオブジェクトの種類によって異なります。オブジェクトによって、文字列を指定するものや、数値を指定するものが分かれています。詳細は下の表を参照してください。
^p
     オブジェクト      : p2の指定
 ----------------------------------------------------------
  ボタン               : ボタン文字列の変更(文字列)
  入力ボックス(数値)   : 入力内容の変更(数値)
  入力ボックス(文字列) : 入力内容の変更(文字列)
  チェックボックス     : チェックのON/OFF(数値)
  コンボボックス       : コンボボックスの内容変更(文字列)
  リストボックス       : リストボックスの内容変更(文字列)
^p
たとえば、チェックボックスを示すIDを指定してから、p2のパラメータに1を指定すると、チェックボックスを強制的にONの状態に変更します。この場合、チェックボックスの内容を保持する変数の値も自動的に書き換えられます。
^
入力ボックスの内容を変更した場合には、自動的に入力ボックスに入力フォーカスが設定され、ボックス内にカーソルが表示されます。

%href
button
input
mesbox
chkbox
listbox
combox
layerobj


%index
objsize
オブジェクトサイズ設定
%group
オブジェクト制御命令
%prm
p1,p2,p3
p1=0〜(64)  : オブジェクトのX方向のサイズ（ドット単位）
p2=0〜(24)  : オブジェクトのY方向のサイズ（ドット単位）
p3=0〜(0)   : Y方向の最低確保行サイズ (ドット単位)

%inst
ボタンや入力ボックスなどを配置する時のオブジェクトの大きさを設定します。
^
p3でボタンやメッセージが置かれた後にカレントポジションが移動する最低量を指定することができます。これにより、ボタンとメッセージを連続して置いた時に同じ大きさでスペースが空くようになります。
^
画面がクリアされると、オブジェクトサイズは自動的にデフォルトに戻ります。

%href
button
chkbox
combox
input
listbox




%index
objsel
オブジェクトに入力フォーカスを設定
%group
オブジェクト制御命令
%prm
p1
p1=0〜(0) : オブジェクトID指定

%inst
p1で指定したオブジェクトIDに入力フォーカスを合わせます。
入力フォーカスを合わせることにより、 mesbox命令やinput命令で配置した入力ボックスの中に入力カーソル(キャレット)を出すことができます。
この命令は、複数の入力ボックスで任意の場所に入力フォーカスを合わせたい場合や、次の入力ボックスにキー入力などで移動するような処理を行なうためのものです。
また、p1に-1を指定した場合は、現在、入力フォーカスが合っているオブジェクトIDをシステム変数statに代入します。



%index
objmode
オブジェクトモード設定
%group
オブジェクト制御命令
%prm
p1,p2
p1=0〜(0) : オブジェクトフォント設定モード指定
p2=0〜1   : フォーカス移動キー指定(0=OFF/1=ON)
%inst
button,input,mesbox などのオブジェクト制御命令で使用されるスタイル等を設定するための命令です。
p1でフォント設定及び、スタイルに関するモードを指定することができます。
これにより、以降のオブジェクト配置命令実行時のスタイル設定が変わります。
モードの値と内容は以下の通りです。
^p
   p1 : マクロ名          : モード
 ------------------------------------------------------------
    0 : objmode_normal      HSP標準フォントを使用
    1 : objmode_guifont     デフォルトGUIフォントを使用
    2 : objmode_usefont     font命令で選択されているフォントを使用
    4 : objmode_usecolor    color命令/objcolor命令の色を使用
^p
ウィンドウが初期化された直後は、モード1に設定されています。
^
モード2に変更した場合は、 font命令で指定したフォントが使われるようになります。これは、オブジェクト制御命令が実行される時点で、設定されているフォントが使用されます。 objmode命令が実行された時点のフォントではないので注意してください。
^
モード4(objmode_usecolor)は、他のモードと加算することで併用ができます。「objmode_usefont+objmode_usecolor」を指定した場合は、どちらのモードも有効になります。
^
p2で、[TAB]キーによるオブジェクトのフォーカス移動モードの ON/OFFを行ないます。 p2を1に指定した場合は、表示されているオブジェクトの入力フォーカスを[TAB]キーで移動することができます。p2のモード指定を省略した場合は、以前のモードを引き継ぎます。
^p
  p2 : モード
 --------------------------------------------------------------
  0  : [TAB]を無効にする
  1  : [TAB]キーによるオブジェクトのフォーカス移動可能(標準)
^p
%href
button
chkbox
combox
input
listbox
objmode_guifont
objmode_normal
objmode_usefont
objmode_usecolor
objcolor



%index
objcolor
オブジェクトのカラー設定
%group
オブジェクト制御命令
%prm
p1,p2,p3
p1,p2,p3=0〜255(0) : 色コード（R,G,Bの輝度）

%inst
オブジェクトが使用する色を設定します。
p1,p2,p3がそれぞれ、R,G,Bの輝度になります。指定する値は、0が最も暗く、255が最も明るくなります。(0,0,0 は黒に、255,255,255 は白になります。)
^
objcolor命令で指定された色は、objmode命令のobjmode_usecolorオプションが指定されている時のみ有効になります。
objmode_usecolorオプションにより、オブジェクト配置時の文字色、背景色を指定することが可能になります。

%href
objmode
color
objmode_usecolor




%index
objenable
オブジェクトの有効・無効を設定
%group
オブジェクト制御命令
%prm
p1,p2
p1=0〜(0) : オブジェクトID指定
p2=0〜(1) : 0ならば無効、0以外は有効

%inst
p1で指定したオブジェクトIDの状態を変更します。
p2で指定した値が0ならば、オブジェクトは無効化されます。
無効化されたオブジェクトは、画面上に存在しますが色が変更され、操作することはできなくなります。
p2で指定した値に0以外を指定すると、通常通りの有効なオブジェクトとなります。
入力ボックスやボタンなど、すべてのオブジェクトを有効・無効化することができます。
%href
objgray

%port-
Let


%index
objskip
オブジェクトのフォーカス移動モードを設定
%group
オブジェクト制御命令
%prm
p1,p2
p1=0〜(0) : オブジェクトID指定
p2=1〜(2) : フォーカス移動モード

%inst
p1で指定したオブジェクトIDのフォーカス移動モードを設定します。
フォーカス移動モードは、[TAB]キーにより配置されたオブジェクトを移動させる場合の挙動をオブジェクトごとに指定するものです。
p2でモード値を指定します。モード値の詳細は、以下の通りです。
^p
  p2 : フォーカス移動モード
 --------------------------------------------------------------
   1 : [TAB]キーにより次のオブジェクトにフォーカスを移動可能(標準)
   2 : [TAB]キーにより次のオブジェクトにフォーカス移動は不可能
   3 : [TAB]キーによるオブジェクトへのフォーカス移動は行なわない(スキップ)
  +4 : フォーカス移動時にテキストを全選択する(入力ボックスのみ)
^p
通常は、オブジェクトを配置した段階で最適なモードが設定されているので、フォーカス移動モードを再設定する必要はありません。
特殊な役割を果たすオブジェクトだけ、フォーカス移動モード変えたり、winobj命令によりシステム定義のオブジェクトを新しく追加した場合に使用してください。
objmode命令により、フォーカス移動モードの機能がOFFにされている場合は、[TAB]キーによるフォーカス移動は行なわれませんので注意してください。

%href
winobj
objmode

%port-
Let


%index
objimage
カスタムボタンの設定
%group
オブジェクト制御命令
%prm
id,x1,y1,x2,y2,x3,y3
id    : カスタムボタンの参照バッファID
x1,y1 : カスタムボタンの参照座標1(通常時)
x2,y2 : カスタムボタンの参照座標2(押し下げ時)
x3,y3 : カスタムボタンの参照座標3(マウスオーバー時)

%inst
カスタムボタンを配置するための設定を行ないます。
カスタムボタンは、button命令で作成される押しボタンのオブジェクト外観を任意の画像に置き換えることができます。
カスタムボタンを作成するためには、あらかじめボタンとして表示するための画像を用意しなければなりません。
^
idパラメーターで、カスタムボタンの画像が格納されている画面バッファIDを指定します。
idパラメーターを省略するか、または-1を指定するとカスタムボタンの設定は無効となり、通常のWindowsシステムが用意するボタンが使用されます。
(x1,y1)に、通常のボタンとして表示する画像の左上座標を指定します。(パラメーター省略時は、(0,0)が指定されます)
(x2,y2)では、ボタンが押された時に、ボタンに表示する画像の左上座標を指定します。(パラメーター省略時は、(0,0)が指定されます)
(x3,y3)では、マウスがボタン上に乗った(マウスオーバー)時、ボタンに表示する画像の左上座標を指定します。(パラメーター省略時は、x1,y1と同様の値が使用されます)
^
objimage命令によってカスタムボタンの設定が行なわれた後は、button命令で配置されるボタンすべてに設定が適用されます。
カスタムボタンは、指定された画像をボタンが配置されている場所にコピーすることで外観を変更しています。
コピーされる範囲は、ボタンと同じ(objsize命令で設定された)サイズとなります。
外観以外の挙動は、通常のボタンと変わりありません。ボタン上の文字表示や、フォーカス移動等も同様にサポートされます。
カスタムボタンの設定は、画面の初期化時(cls命令やscreen命令実行時)にクリアされます。

%href
button

%port-
Let


%index
layerobj
レイヤーオブジェクトを追加
%group
オブジェクト制御命令
%prm
p1,p2,p3,*label,p4
p1,p2     ; レイヤーオブジェクトのXYサイズ（ドット単位）
p3(0)     : レイヤーID
*label    : レイヤーオブジェクトの処理サブルーチン
p4=0〜(0) : オプション値

%inst
カレントポジションにレイヤーオブジェクトを配置します。
レイヤーオブジェクトは、ユーザーによって定義される配置オブジェクトです。
あらかじめ画面上の描画を行うサブルーチンを登録しておくことで、指定したタイミング(レイヤー)で描画を実行させることができます。
レイヤーオブジェクトに描画領域を通知するため、p1,p2でX,Yサイズをドット単位で指定する必要があります。p1,p2の指定を省略した場合は、画面全体のサイズとなります。
これで、カレントポジションX,Yを左上の座標として、p1,p2で指定したサイズまでをレイヤーオブジェクトとして扱います。
p3パラメーターで、描画するレイヤーを指定します。
^p
  p3 : 描画するレイヤー
 --------------------------------------------------------------
    0 : 描画を行わない(objlayer_min)
    1 : 背景レイヤー(objlayer_bg)
    2 : 通常の描画レイヤー(objlayer_normal)
    3 : GUIレイヤー(objlayer_posteff)
    4 : 最前面の描画レイヤー(objlayer_max)
 +256 : レイヤーオブジェクトの重複登録を許可する(objlayer_multi)
^p
*labelパラメーターで、レイヤーオブジェクトの処理サブルーチンを指定します。
このサブルーチンはユーザー自身が用意する必要があります。
p4パラメーターで、レイヤーオブジェクトに設定する任意の整数値を設定することができます。
^
レイヤーオブジェクトが何を行うかはすべてユーザーが作成したサブルーチンに任されています。
サブルーチンは、システム変数iparam,wparam,lparamが設定された状態で呼び出されます。これらの情報をもとに、レイヤーオブジェクトの描画を含めて、ユーザーが処理を記述してください。
レイヤーオブジェクトの処理サブルーチン内では、描画のみを行ってすぐにreturn命令で処理を終了させる必要があります。wait/await/redrawなどのタスクが停止する命令は記述できません。
レイヤーオブジェクトについての詳しい使用方法は、プログラミングマニュアルを参照してください。


%href
objprm

%port-
Let


