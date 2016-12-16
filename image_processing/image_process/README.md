A rubik's cube solver written in python 2.7 using OpenCV using your webcam.

NOTE: cube uses color detection and color detection is very hard to fix for
every possible situation, because certain light influences a color detector and the
color scheme on a rubik's cube. So future users need to adjust the HSV boundaries so that the code can work properly. 

```

# Usage

Run cube:
python cube.py

```
```

This opens a webcam interface where you see basically the above photo.
You have 4 things:

* The 9 center squares.

These are used for scanning in
your cube colors.

* The 9 stickers in the upper left corner.

These will update
immediately how the computer sees the colors.

* The seconds 9-sticker display below the one in the upper left corner.

When pressing `space` a 3rd cube template updates below the one in the upper left corner.
This is the state that is saved, so you know how qbr saved it.

* Amount of sides scanned

This is not shown in the above demo image, but in the bottom left corner is shown
the amount of sides scanned. This is so you know if you've scanned in all sides before
pressing `esc`.

### Keybindings:

`space` for saving the current view

`esc` for quit.

Cube checks if you have filled in all 6 sides when pressing `esc`.
