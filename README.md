# Simple visualizer of the ear clipping triangulation algorithm #

## Dependencies ##
```
Boost >= 1.54
Qt
OpenGL
gcc >= 4.6
```

## Building application ##
Clone this repository, `cd` into it and type following commands:
```
git submodule update &&
cd visualization &&
qmake && make
cd ..
qmake && make
```

## Application features ##

```
Add point:           double click
Run triangulation:   press key 'Enter'
Save points:         press key 'S'
Load points:         press key 'L'
Delete points:       press key 'D'
Back to draw mode:   press key 'Escape'
```

