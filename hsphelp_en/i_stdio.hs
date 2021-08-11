;
;HELP source file for HSP help manager
;(Lines beginning with; are treated as comments)
;

%type
Built-in instructions
%ver
3.6
%note
ver3.6 standard instruction
%date
2019/12/12
%author
onitama
%url
http://hsp.tv/
%port
Win
Cli
Let




%index
getkey
Key input check
%group
Basic I / O control instructions
%prm
p1,p2
p1 = variable: variable to read
p2 = 1 ~ (1): Key code

%inst
Check the state of the keyboard and mouse buttons and assign them to variables. If the specified button is pressed, 1 is assigned, and if it is not pressed, 0 is assigned.
The details of the value specified by the key code are as follows.
^p
  Keyed code: actual key
 ------------------------------------------
        1: Left mouse button
        2: Right mouse button
        3: Cancel ([CTRL] + [BREAK])
        4: 3 button Button in the middle of the mouse
        8: [BACKSPACE] (PC98 [BS])
        9    : [TAB]
       13    : [ENTER]
       16    : [SHIFT]
       17    : [CTRL]
       18: [ALT] (PC98 [GRPH])
       20    : [CAPSLOCK]
       27    : [ESC]
       32: Space bar
       33: [PAGE UP] (PC98 [ROLLDOWN])
       34: [PAGEDOWN] (PC98 [ROLLUP])
       35: [END] (PC98 [HELP])
       36: [HOME] ([HOMECLR] of PC98)
       37: Cursor key [<-]
       38: Cursor key [↑]
       39: Cursor key [->]
       40: Cursor key [↓]
   48 to 57: [0] to [9] (main keyboard)
   65-90: [A]-[Z]
  96 to 105: [0] to [9] (numeric keypad)
 112 ~ 121: Function keys [F1] ~ [F10]
^p
You may be able to obtain keys other than those listed in this table. (You can look up the keycode by running the sample.)


%sample
title "Please enter the key (^^) v"
onkey *inkey
onclick *inkey
stop
*inkey
if lparam>>30:stop
mes iparam
stop


%href
stick




%index
mouse
Mouse cursor coordinate setting
%group
Basic I / O control instructions
%prm
p1,p2,p3
p1, p2: X, Y coordinates to set
p3 (0): Setting mode value
%inst
Changes the mouse cursor to the coordinates specified by p1 and p2.
The coordinates you specify are those on the display (usually X = 0 to ginfo_dispx / Y = 0 to ginfo_dispy) rather than the coordinates in the window.
If you omit the specifications of p1 and p2, the current coordinates will be applied as they are.
The value of p3 determines the setting mode for showing / hiding the mouse cursor. The value of p3 works as follows.
^p
Mode value Contents
-------------------------------------------------------------------------
    0 Hide the mouse cursor when the value of p1 or p2 is negative
          At other times, move the mouse cursor to the coordinates (p1, p2) to display.
   Move the mouse cursor to the coordinates of -1 (p1, p2) to hide it.
    Move the mouse cursor to the coordinates of 1 (p1, p2) and keep the show / hide settings.
^p
If the value of p3 is 0 (default value), set the mouse cursor display to OFF (hide) on the HSP window when p1 or p2 is a negative value.
After turning off the display of the mouse cursor, it will be displayed again by setting valid coordinates.
If the value of p3 is other than 0, you can specify the coordinates regardless of the value of (p1, p2) and the ON / OFF of the mouse cursor. Use this when you need to handle negative coordinates, such as in a multi-monitor environment.

%port-
Let




%index
randomize
Initialization of random number generation
%group
Basic I / O control instructions
%prm
p1
p1 = 0 ~ (indefinite): Random number initialization parameter

%inst
Initializes the random number pattern generated by the rnd function.
^
Random numbers initialized by specifying the same value for p1 will always generate random numbers in the same pattern.
If p1 is omitted, it will be initialized using an indefinite value obtained from the Windows timer. This allows you to generate a completely different random number each time.

%href
rnd




%index
stick
Get keystroke information
%group
Basic I / O control instructions
%prm
p1,p2,p3
p1 = variable: variable to read
p2 = 0 to (0): Non-trigger type key specification
p3 = 0 to 1 (1): Window active check ON / OFF

%inst
Check the states of commonly used keyboard and mouse buttons together and assign them to variables.
^
When the stick instruction is executed, the following multiple button information is assigned as one numerical value to the variable specified by p1.
^p
Value (decimal) Value (hexadecimal) key
-------------------------------------
     1: $ 00001: Cursor key left (<-)
     2: $ 00002: On the cursor key (↑)
     4: $ 00004: Cursor key right (->)
     8: $ 00008: Cursor down (↓)
    16: $ 00010: Spacebar
    32: $ 00020: Enter key
    64: $ 00040: Ctrl key
   128: $ 00080: ESC key
   256: $ 00100: Left mouse button
   512: $ 00200: Right mouse button
  1024: $ 004000: TAB key
  2048: $ 000080: [Z] key
  4096: $ 01000: [X] key
  8192: $ 02000: [C] key
 16384: $ 04000: [A] key
 32768: $ 08000: [W] key
 65536: $ 10000: [D] key
131072: $ 20000: [S] key
^p
If no button is pressed, 0 is assigned.
^
If multiple buttons are pressed at the same time, all of those numbers will be added.
For example, if the right cursor key and the spacebar are pressed at the same time, 20 of 4 + 16 will be read into the variable.
The operator "&" is useful when checking this numerical value with an if instruction.
^p
example :
stick a, 0; read key state into variable a
if a & 16: goto * spc; Was space pressed?
if a & 32: goto * ent; Was Enter pressed?
^p
In this way, only one key information can be extracted from the numerical value that contains multiple button information in "Variable & Key Information".
^
The stick instruction is very similar to the getkey instruction, except that it only detects the moment the button is pressed. In other words, when the button is pressed, the information that is pressed only once is returned, and after that, it is not pressed until the pressed button is released.
^
However, the non-trigger type key specification on p2 makes it possible to detect information even if it is held down.
^
If you specify a key code for p2 that can be detected even if you hold it down (the code in the table above. If you have more than one, add each number), only that key will be detected as long as the button is pressed. It will be so.
^
This instruction looks very complicated, but it can be a very useful feature when writing scripts with keys.
For example, imagine a shooter. Your aircraft must move up, down, left and right as long as the button is pressed. However, the next missile will not be fired until the missile launch button is pressed once.
In such a case, only the up / down / left / right keys should be specified as non-trigger type keys, and the other keys should be returned only once.
^
You can also turn on / off the function that disables input when the window is not active on p3.
If p3 is 1 or omitted, keystrokes will be disabled if the HSP window is not active.
If p3 is 0, then keystrokes are made under all circumstances.


%href
getkey
jstick



%index
logmes
Send debug message
%group
HSP system control instructions
%prm
"message"
"message": Message to log

%inst
When the debug window is displayed, the contents of "message" are recorded in the debug log.
It can be used for the contents of variables at a certain point in time and for checking passage.
It is necessary to set the display mode of the debug window from the script editor or to display the debug window by the assert command.
This instruction has no effect when creating an executable file.

%href
assert



%index
assert
Debug window display
%group
HSP system control instructions
%prm
p1
p1 (0): Conditional expression during debugging

%inst
Temporarily suspend the program and display the debug window.
If a conditional expression is specified for p1, the debug window will be displayed only if the condition for p1 is incorrect.
(Please note that the conditional expression when passing is written in p1.)
^p
assert a> 5; debug when a is 5 or less
^p
This instruction has no effect when creating an executable file.

%href
logmes
%port-
Let



%index
mcall
Method call
%group
Basic I / O control instructions
%prm
p1, p2, p3…
p1: Variable name
p2: method name
p3: Parameter

%inst
Calls the method according to the type of the variable specified in p1.
It is possible to call COM automation methods by specifying the COM object type in the variable of p1.
Specify the method name (character string) or dispatch ID (DISPID) in p2, and specify the argument after p3.
The number of parameters after p3 and the type are converted appropriately and passed to the method.
The return value of the result of executing the method is assigned to the variable set by the results instruction.
Also, if the method execution is successful, the system variable stat becomes 0, and if an execution error occurs, the result code (HRESULT) is assigned to the system variable stat.

The mcall instruction can provide new functionality by providing extended variable types. By default, only COM object types are supported.

%href
#usecom
newcom
delcom
querycom
comres
%port-
Let



%index
setease
Set the calculation formula of the easing function
%group
Basic I / O control instructions
%prm
p1,p2,p3
p1: Minimum output value (real value)
p2: Maximum output value (real value)
p3: Type value of formula
%inst
Set the easing function that interpolates the numerical values in the specified range with an arbitrary calculation formula.
The settings specified here will be reflected when the easing function (getease, geteasef) gets the value.
The following can be specified for the type value of the formula.
^p
Macro name Interpolation content
	------------------------------------------------------------
ease_linear Linear (linear interpolation)
ease_quad_in Acceleration (Quadratic)
ease_quad_out Deceleration (Quadratic)
ease_quad_inout Acceleration-> Deceleration (Quadratic)
ease_cubic_in Acceleration (Cubic)
ease_cubic_out Deceleration (Cubic)
ease_cubic_inout Acceleration-> Deceleration (Cubic)
ease_quartic_in Acceleration (Quartic)
ease_quartic_out Deceleration (Quartic)
ease_quartic_inout Acceleration-> Deceleration (Quartic)
ease_bounce_in Bounce effect (on)
ease_bounce_out Bounce effect (out)
ease_bounce_inout Bounce effect (in / out)
ease_shake_in Shake effect (on)
ease_shake_out Shake effect (out)
ease_shake_inout Shake effect (in / out)
ease_loop Interpolation loop (*)

The type indicated by (*) can be added to other types.
^p
If the formula type value is omitted, the previously set value is used.

The easing function supports basic calculations to get an animation of natural movement.
For example, suppose you want to move an object that had an X coordinate of 100 to an X coordinate of 200 with a 50-frame animation.
Normally, the animation is created by adding 2 X coordinates such as 100, 102, 104, 106, etc. for each frame to obtain new coordinates.
However, this is only a linear movement. The easing function can get the coordinates for each frame from a specific formula.
By setting the formula, you can realize organic animation such as starting movement slowly, moving while accelerating, decelerating again in front of the target, moving like a parabola and swinging (shaking). It can be used for various purposes such as movement.
To use the easing function, first specify the range in which the value changes with the setease instruction and the calculation formula.
^p
; Setting of easing function
	setease 100,200,ease_cubic_inout
^p
In the above example, the easing function to get the value from 100 to 200 is set by the formula of ease_cubic_inout.
Then get the actual value with the getease or geteasef function.
getease and geteasef are basically the same, but the values obtained are either integers or real numbers.
When dealing with normal coordinates, there is no problem even if you get it as an integer value. (Both are calculated by real numbers inside the easing function)
^p
; Setting of easing function
	setease 100,200,ease_cubic_inout
	i=0
	repeat
		redraw 0
color 0,0,0: boxf; clear screen
x = getease (i, 50); Get easing value (integer)
		color 255,255,255
pos x, 100: mes "●"
		redraw 1
		await 30
		i=i+1
	loop
^p
The argument of the getease function is getease (time elapsed value, maximum value).
The elapsed time value is an integer value starting from 0 and specifies up to the value specified by the maximum value.
In other words, in the above example, getease (0,50) returns the start value in the range of 100 to 200 set by the setease instruction, that is, 100.
As the time elapsed value increases, so does the value returned from 100 to 200. And the formula is such that 200 is returned when getease (50,50) is reached.
^p
When the elapsed time value is 0 = The minimum output value specified by the setease instruction is returned.
When the elapsed time value is the maximum value = The maximum output value specified by the setease instruction is returned.
^p
If you omit the maximum parameter, 4096 is used.
In the case of the geteasef function, the argument is the same as geteasef (time elapsed value, maximum value).
However, you can use real numbers for both the elapsed time value and the maximum value, and you can use the easing function with finer precision. Also, if you omit the maximum parameter in the geteasef function, 1.0 is used.

Normally, if the elapsed time value is negative, it is considered to be 0. Also, if the elapsed time value exceeds the maximum value, it is treated as the maximum value.
However, if ease_loop (interpolation loop) is added in the type setting of the calculation formula by the setease instruction, the interpolation loop (repetition) is performed including the values outside the range. In the interpolation loop, the movement goes back and forth between the minimum output value and the maximum output value according to the elapsed time value.

The easing function is a convenient function that allows you to easily use advanced animation by mastering it, although it may be difficult to imagine the result until you get used to it.
Also, because it is a built-in instruction as standard, it can be called in any runtime such as HSP3Dish or HGIM G4 as well.

%href
getease
geteasef
%port-
Let



%index
sortval
Sort array variables numerically
%group
Basic I / O control instructions
%prm
p1,p2
p1: Numeric array variable name
p2: Sort order (0 = small order / 1 = large order)
%inst
Sorts (sorts) array variables that store numbers in the specified sort order.
Directly sorts the array variables specified by p1.

Information about sorting can be obtained with the sort get command.
%href
sortget
%port-
Let


%index
sortstr
Sort array variables by string
%group
Basic I / O control instructions
%prm
p1,p2
p1: String type array variable name
p2: Sort order (0 = small order / 1 = large order)
%inst
Sorts (sorts) the array variables that store the character strings in the specified sort order.
Directly sorts the array variables specified by p1.
The sort order is sorted by ASCII code large and small, so it can be sorted in alphabetical order or alphabetical order.

Information about sorting can be obtained with the sort get command.
%href
sortget
%port-
Let



%index
sortnote
Sort memory note strings
%group
Basic I / O control instructions
%prm
p1,p2
p1: Memory note format string type variable name
p2: Sort order (0 = small order / 1 = large order)
%inst
Memory Sorts variables that contain notepad-style strings in the specified sort order.
Directly sorts the variables specified by p1.
The sort order is sorted by ASCII code large and small, so it can be sorted in alphabetical order or alphabetical order.

Information about sorting can be obtained with the sort get command.
The memory notepad format is a data string separated by "\\ n" (line feed code) that can be used with notesel, noteget commands, etc.
%href
notesel
sortget
%port-
Let



%index
sortget
Get index of sort source
%group
Basic I / O control instructions
%prm
p1,p2
p1: Variable name to which the result is assigned
p2: Index No.
%inst
In the array after executing the sortstr, sortval, sortnote instructions, the stored data is checked to which index it was placed before sorting and the result is returned.
For example, after sorting the array variable a, if the command sortget n, 4 returns a value of 1, the value currently contained in the array variable a (4) will be a (before sorting. Indicates that it was in 1).
This command is useful when you want to sort only a part of the data and sort other data based on that information.
%href
sortstr
sortval
sortnote
%port-
Let



