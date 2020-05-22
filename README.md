# 4D Rendering Engine

## Introduction
This OpenGL program in C++ is capable of rendering edges of the "shadow" of arbitrary 4-dimensional [polytopes](https://en.wikipedia.org/wiki/Polytope)
to the screen. The program utilizes no builtin matrix operations, quaternion transforms, etc. All math that has been implemented
was derived by myself with limited outside knowledge of linear algebra or graphics programming, other than the basic concepts of
spherical coordinate systems and the parametric equations of lines and planes in n-space. This was written as a personal excercise
and proof of concept, rather than to create an efficient and optimized rendering system.

## How It Works
In 4-dimensional space, a fourth coordinate axis exists that is orthogonal to the x, y, and z axes. As 3-dimensional beings, it's not
really feasible to perceive what this would look and feel like, but since the applicable math works out so nicely when extrapolating up a
dimension, a program like this one can give a glimpse into how geometries in higher dimensions would behave. Just about anything 
regarding 4 dimensions can be understood, if not intuitively then logically, by creating an analogy that applies to 3 or fewer dimensions. If you couldn't, 
it wouldn't really make much sense to spend time thinking about something so abstract and intangible. This thought process might be
helpful in seeing how this program works, and how one (me) might have gone about creating it from the ground up.

### Thought Excercises
Suppose you want to gain a better understanding of how rotation works in 4 dimensions. Start by creating an analogy, if one exists, in
3D. Three dimensional objects can rotate—check. How? No matter how a 3D object rotates, it rotates about a (3 - 2)D = 1D pivot (a 
line). Now extrapolate back up—a 4D object must rotate about a (4 - 2)D = 2D pivot (a plane). To give you more confidence in your
assumption, notice that a 2D object also always rotates around a (2 - 2)D = 0D pivot (a point). The assumption must be correct.

To take it a step further, imagine a 2D
square lying in a 2D reality. If a 3D being could manipulate that square (think of it like a playing card on a table), it could flip it 
into the 3rd dimension, keeping one of its edges in the 2D plane, and lay it down flat again. To a 2D being, this would look like pure
magic. To them, it would look like a cube just dissappeared except for one face and reappeared on the other side of that face as its
mirror image. You know where this is going. Similarly, a 4D being could rotate a 3D cube into the 4th dimension, keeping one of its six
2D faces in the 3D hyperplane of our reality, and make it reappear on the other side of that face as its mirror image, without the face
ever moving.

Another example is understanding how a 4D being might see. Whereas a 3D human's retina is 2D, and a 2D being could have no more
than a 1D retina, a 4D being's retina would be 3D. It could perceive every 2D cross-section of our 3D reality at once, the same
way a human could see every 1D cross-section of a 2D reality. If that sounds crazy, consider a 1D being (living on a line) that 
would probably think it's equally crazy (if a stick is capable of thought) that a 2D being would be capable of seeing its entire
reality at once. (This is a critical insight into how this program's `Camera4d` really works.)

### `Camera3D`
It is not difficult then to understand how this program's `Camera4D` projects points from 4D space down into 3D 
space (and subsequently to 2D space to be rendered on the screen) by first understanding how a `Camera3D` works. Objects 
of the `Camera3D` class have a `location`, which is an (x, y, z) coordinate that represents where the camera is. However, this gives
no information about the direction the camera is facing. A `spatialVector` (a mathematical vector, with a 3D direction and a
magnitude) called the `normal` specifies this direction. The camera's `focus` is a special point that is always located away from
the camera's `location` in the direction that faces opposite the `normal` by the camera's `focalDistance`. The `Camera3D` class 
has methods `getUnitUpVector()` and `getUnitRightVector()` that return unit `spatialVector`s pointing up and right relative to the camera. 
(A side note: the 3D camera cannot [roll](https://en.wikipedia.org/wiki/Aircraft_principal_axes), as is standard in most video games,
so the rightward-facing vector is always orthogonal to the vertical z axis.)

The `Camera3D` is able to render `point3d`s—that is, cast a `point3d` to a `point2d` based on the location of the point and the
camera's `location`, `focus`, and `normal`. In short, `Camera3D`'s `projectPoint()` method takes a `point3D` (call it `P`) and solves
for the intersection point `I` of the line between the `focus` and `P` and the 2D plane orthogonal to the camera's `normal` that
passes through its `location`. This still gives a point in 3-space. To find the (x, y) coordinate that represents the point's
location on the camera's rendering plane, analygous to the screen's (x, y) pixel location where the point should be rendered, the vector
from the camera's `location` to `I` (which is always parallel to the rendering plane) is scalar-projected onto the vectors given by
`getUnitRightVector()` (to give the x coordinate) and `getUnitUpVector()` (to give the y coordinate).

Since scalar projections give positive or negative values depending on the angle between the vectors, the resulting `point2d` is 
treated as being relative to the screen's center as (0, 0) when an `edge` containing the point is rendered. Additionally, the
(x, y) coordinates to render are scaled by `ORTHO_ZOOM`, a constant in `utils.cpp` that represent how many screen pixels equal
one unit in the `Scene`'s 3D space.

### `Camera4D`
Everything about how the `Camera4D` works is exactly analogous to `Camera3D`. It has a `location` (`point4d`), `focus` (`point4d`), 
and `normal` (4D `spatialVector`). To render a 4D point `P` to 3D space, it first finds the 4D intersection point `I` of the line 
between the `focus` and `P` and the 3D hyperplane orthogonal to the `normal` that passes through the `location`. It then 
scalar-projects the 4-dimensional `I` into 3D space by creating a vector from the `location` to `I` and scalar-projecting it onto 
the vectors given by `getUnitRightVector()` (to give the x coordinate), `getUnitUpVector()` (to give the y coordinate), and 
`getUnitOutVector()` (to give the z coordinate). 

## Rendering Different Objects
The `main()` function, found in `graphics.cpp`, currently has hard-coded values for the vertices of the x, y, and z axes, as well
as those for a 3d cube and 4d cube. To create a new `Object`, instantiate a new `Object3D` using a `vector<point3d>` and 
`vector<edge3d>`, or an `Object4D` using a `vector<point4d>` and a `vector<edge4d>`. An `Object` must have at least one edge to
render on the screen, and will not render any `point`s not contained in at least one `edge`.

Currently, there is no better way than to hard-code vertex locations before instantiating an `Object`. However, an `extrude()`
method is nearly implemented (found in `utils.cpp`) that will extrude new vertices in the direction of a given `spatialVector`
and add new `edge`s to the `Object` appropriately.

For now, to add a custom `Object`, follow the format as utilized in `main()`.

<i>Note: Datatypes such as `edge`, `point`, and `spatialVector` are structs implemented in `utils.cpp`.</i>

## Movement and Rotation
A `Scene`, which has a vector of all `Objects` being rendered, has both a `Camera3D` and a `Camera4D`, whose movements are controlled
independently. When the program begins, the movement and rotation keybinds control the 3D camera. To view a 3D object from a
different perspective, move using the 3D camera. To change how the vertices of a 4D object are cast into 3 dimensions, move using
the 4D camera. <b>To toggle which camera is being controlled, press `t`.</b>

The keyboard controls are set up as follows.

### 3D Camera
Movement keys are standard WASD, with rotation keys being IJKL.
  - Forward: `w`
  - Left: `a`
  - Back: `s`
  - Right: `d`
  - Up: `space`
  - Down: `left shift`
  - Rotate up: `i`
  - Rotate left: `j`
  - Rotate down: `k`
  - Rotate right: `l`
### 4D Camera
Four dimensional movement needs additional keys for moving and rotating into the 4th dimension—moving and rotating "in" and "out".
<b>The same keybinds as above apply</b>, with these additional ones:
  - In: `q`
  - Out: `e`
  - Rotate in: `u`
  - Rotate out: `o`

<sub><sup>Copyright © 2020 Jackson Hall. All rights reserved.</sup></sub>
