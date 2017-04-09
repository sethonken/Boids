Boids by Seth Onken

*str2int function was taken from http://stackoverflow.com/questions/16388510/evaluate-a-string-with-a-switch-in-c

Program will open 3 windows: Boids, Interface, and the .exe window.

The Boids window has a menu attached to mouse right:
	<Define Circle Obstacle> - Define a circle by clicking mouse left, dragging the mouse until desired circle size, then releasing mouse left.
	<Define Line Obstacle> - Define a line by clicking mouse left for the start point and then clicking mouse left for end point.
	<Delete Obstacle> - Clicking on any obstacle will delete it.
	<Delete All Obstacles> - Clears display of all obstacles.
	<Single-Step> - The display will pause and wait for user input in the .exe window. "SPACE" for one time step or "Enter" to resume display and enable menus.
	<Restart> - Resets boids to starting point.
	<Exit> - Closes program.

The interface window has 8 sliders and 3 buttons. 
	The sliders control various parameters:
		<Boid Number> - Number of boids to run at once.
		<Radius Repulsion> - Size of radius around boid that it orients its direction away from other boids.
		<Radius Orientation> - Size of radius around boid that it orients its direction with other boids.
		<Radius Attraction> - Size of radius around boid that it orients its direction to other boids.
		<Time Step> - Distance boids travel per frame.
		<Perception Angle> - Amount of degrees around boid that it can see other boids.
		<Turn Angle> - Size of angle turn that a boid takes when turning in relation to other boids.
		<Strength of Goal> - How much boids are attracted to goal.
	The button <Click Here to input> allows user input into the .exe window to set a random integer seed.
	The button <Load> allows user to load a saved run by entering the file name into the .exe window.
	The button <Save> allows user to save a run by entering the file name into the .exe window.

If boids travel through obstacles, consider that the <Strength of Goal> is probably set too high or that the obstacles are set too close together.

Included are 5 demo files that showcase various settings and obstacles courses: "demo1" -> "demo5"