# shadows-are-walls-REVAMP
This is a rewrite of my original shadows-are-walls project. Runs on (mostly) pure ECS in Unity, and has a fully custom physics engine. 

The goal of this project is to make a puzzle-platformer game which revolves around the mechanic of shadows being solid. This mechanic turns out to have some really interesting (and confusing) behavior when you take the idea to its extremes. My hope for the completed game is a collection of engaging puzzles that gradually let the player figure out how the physics works. Ideally, the game would require the player to think in a way that is totally unique and have realizations that are very satisfying.

The current state of the project is a mostly-implemented game engine, with some levels designed. There are still many game design problems that must be resolved in order to make this engine into a good game.


## Features

- Efficient physics engine built from scratch
- Physics debugger that shows how impulses are applied during sub-steps within a frame
- Springs
- Pixel perfect shadow rendering (using geometry shaders)
- Player physics (running and jumping)

## Capabilities of the Physics Engine

### Boxes can stack stably

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/1.gif)
![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/2.gif)

### Solid shadows

Orange objects cast shadows because they are opaque. White objects cannot penetrate shadows.

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/3.gif)

### Lights can respond to collisions

If an object is penetrating a shadow, there are two ways to resolve the penetration. Move the box, or move the shadow (by moving the light). Lights are able to rotate to resolve collisions if their moment of inertia is not set to infinity, as in this example.

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/4.gif)

### Boxes can collide with light corners

When two lights overlap, they form a corner of shadow. Collisions with these corners are supported. However, friction during these collisions is not yet implemented.

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/5.gif)

### Shadow corners can react to collisions

If a box hits a shadow corner and the two lights involved are able to move, the shadow corner respond in a way that "makes sense." In the example, the white block is pushing down on the two shadow corners, so the corners move down (by rotating the lights towards the center.)

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/6.gif)

### Opaque objects can move

If an object is penetrating on the edge of a shadow casted by an opaque object, the opaque object can move in such a way that the collision is resolved. These collisions are robust enough to allow for stacking behavior.

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/7.gif)
![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/8.gif)
![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/9.gif)

### Shadow corners from opaque objects can react to collisions

This is the part where it starts to get confusing. If two lights are shining on an opaque object, they form two shadows. But the only solid part of the shadow is where the two shadows overlap. This creates a triangle of shadow that comes out of the opaque object. And this shadow is able to react to collisions in a way that "makes sense." The way the math worked out caused there to be a force on the opaque object such that it is pushed to the lowest possible point, similar to a ball rolling down a hill. The simulation looks so slippery because friction is not implemented for shadow corners yet.

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/10.gif)
![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/11.gif)

### Shadow slices can be resolved robustly

The engine is able to handle shadow corners with extremely small angles, that will quickly "slice" through objects. In the first two examples, the two lights are arranged so that the shadow slices from the left and from the from the right. In the last example, the lights are at the exact same position. 

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/12.gif)
![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/13.gif)
![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/13.5.gif)

### Shadow triangles can react to collisions

This might be the most confusing physical property of the engine. If you have three shadow edges that overlap, you can have a situation where there is a small triangle of shadow. Such a triangle can open up inside of an object. The engine is able to elegantly resolve that situation by "cinching" the triangle shut.

![](https://github.com/EricPendergast/shadows-are-walls-REVAMP/blob/main/Gifs/14.gif)
