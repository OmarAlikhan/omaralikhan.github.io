/**
 * Class used to emulate the Pololu Zumo 32u4 robot.
 * @class Tracy
 * @constructor
 */
function Tracy(x,y,z)
{
    /* Physical body of Tracy along with its corresponding shape */
    this.chassisShape = new CANNON.Box(new CANNON.Vec3(0.9, 0.7,0.2));
    this.chassisBody = new CANNON.Body({ mass: 5 });
    this.chassisBody.position.set(x,y,z);
    this.chassisBody.quaternion.setFromAxisAngle(
        new CANNON.Vec3(0,0,1), THREE.Math.degToRad(180));
    this.chassisBody.addShape(this.chassisShape);

    /* Settings supported by Cannon JS RaycastVehicle class */
    var options = {
        radius: 0.5,
        directionLocal: new CANNON.Vec3(0, 0, -1),
        suspensionStiffness: 30,
        suspensionRestLength: 0.3,
        frictionSlip: 1,
        dampingRelaxation: 2.3,
        dampingCompression: 4.4,
        maxSuspensionForce: 100000,
        rollInfluence:  0.01,
        axleLocal: new CANNON.Vec3(0, 1, 0),
        chassisConnectionPointLocal: new CANNON.Vec3(1, 1, 0),
        maxSuspensionTravel: 0.3,
        customSlidingRotationalSpeed: -30,
        useCustomSlidingRotationalSpeed: true
    };

    /* Create the vehicle and assign its chassisBody */
    this.vehicle = new CANNON.RaycastVehicle({ chassisBody: this.chassisBody });

    /* Add four wheel bodies to the vehicle */
    options.chassisConnectionPointLocal.set(0.6, 0.9, 0.2);
    this.vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(0.6, -0.9, 0.2);
    this.vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(-0.6, 0.9, 0.2);
    this.vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(-0.6, -0.9, 0.2);
    this.vehicle.addWheel(options);

    /* Create each of the wheels by creating its shape */
    this.vehicle.wheelBodies = [];
    for(var i=0; i<this.vehicle.wheelInfos.length; i++){
        var wheel = this.vehicle.wheelInfos[i];
        var cylinderShape = new CANNON.Cylinder(wheel.radius, wheel.radius, wheel.radius / 2, 20);
        var wheelBody = new CANNON.Body({ mass: 0 });
        wheelBody.type = CANNON.Body.KINEMATIC;
        wheelBody.collisionFilterGroup = 0;
        var q = new CANNON.Quaternion();
        q.setFromAxisAngle(new CANNON.Vec3(1, 0, 0), Math.PI / 2);
        wheelBody.addShape(cylinderShape, new CANNON.Vec3(), q);
        this.vehicle.wheelBodies.push(wheelBody);
    }

    /* Calculate acceleration and update its readings every 500ms */
    this.acceleration = new CANNON.Vec3(0,0,0);
    var previousVelocity = new CANNON.Vec3(0,0,0);
    function updateAccelerometerReadings()
    {
        this.chassisBody.velocity.vsub(previousVelocity,this.acceleration);
        previousVelocity.copy(this.chassisBody.velocity);
    }
    setInterval(updateAccelerometerReadings.bind(this),500);

    /* Update proximity sensors readings every 500 ms */
    this.proximitySensorReading = {
        left: 0,
        right: 0,
        front: 0
    }
    function updateProximitySensorReadings()
    {
        var updatedReading = this.getProximitySensorReading();
        this.proximitySensorReading.front = updatedReading.front;
        this.proximitySensorReading.left = updatedReading.left;
        this.proximitySensorReading.right = updatedReading.right;
    }
    setInterval(updateProximitySensorReadings.bind(this),500);


    /* Update gyroscope sensor readings every 500 ms */
    this.orientation = {
        x: 0,
        y: 0,
        z: 0
    }
    function updateGyroscopeReadings()
    {
        var updatedOrientation = new CANNON.Vec3(0,0,0);
        this.chassisBody.quaternion.toEuler(updatedOrientation);
        this.orientation.x =  THREE.Math.radToDeg(updatedOrientation.x);
        this.orientation.y =  THREE.Math.radToDeg(updatedOrientation.y);
        this.orientation.z =  THREE.Math.radToDeg(updatedOrientation.z);
    }
    setInterval(updateGyroscopeReadings.bind(this),500);
}

/** 
 * Returns the acceleration of Tracy at the time the function is called.
 * @method getAccelerometerReading
 * @return {Object} Returns a CANNON.Vec3 object which represents the acceleration measured.
 *                  The acceleration has an x, y and z components that can be accessed.
 */
Tracy.prototype.getAccelerometerReading = function()
{
    return this.acceleration;
}

/**
 * Returns the angular velocity of Tracy at the time the function is called.
 * @method getGyroscopeReading
 * @return {Object} Returns a CANNON.Vec3 object which has .x .y .z properties.
 */
Tracy.prototype.getGyroscopeReading = function()
{
    return this.chassisBody.angularVelocity;
}

/**
 * Returns the orientation of Tracy as a quaternion.
 * @method getMagnetometerReading
 * @return {Object} Returns a CANNON.Vec3 object which represents the orientation measured.
 *                  The quaternion has an x, y and z components that can be accessed.
 */
Tracy.prototype.getMagnetometerReading = function()
{
    return this.chassisBody.quaternion;
}

/** 
 * Apply the a brake force to all the wheels. It is not sufficient enough
 * to set the engine force to zero as the wheels will continue to roll
 * on the surface. Therefore, use this function to fully stop the vehicle.
 * @method stop 
 */
Tracy.prototype.stop = function()
{
    /* Loop over all the wheels and apply a brake force to each */
    /* setBrake(force,wheelIndex) */
    for(var i=0; i<4; i++) { this.vehicle.setBrake(10,i); }
}

/**
 * Set the left and right motor speeds. 
 * Tracy can rotate to either side by setting one motor speed to x and the other to -x.
 * @method setMotorSpeeds
 * @param {Integer} amount of force to be applied on the right motor measured in N.
 * @param {Integer} amount of force to be applied on the left motor measured in N.
 */
Tracy.prototype.setMotorSpeeds = function(rightMotorSpeed,leftMotorSpeed)
{
    /* The physics engine has to explicitly set the brakes in order
     to stop Tracy from continuing to roll across the surface. */
    if( (rightMotorSpeed === 0) && (leftMotorSpeed === 0) ) { this.applyBrakes(5); }

    // The wheels numbered 1 and 3 are on the left side of Tracy.
    this.vehicle.applyEngineForce(leftMotorSpeed, 1);
    this.vehicle.applyEngineForce(leftMotorSpeed, 3);

    // The wheels numbered 0 and 2 are on the right side of Tracy.
    this.vehicle.applyEngineForce(rightMotorSpeed, 0);
    this.vehicle.applyEngineForce(rightMotorSpeed, 2);
}

/**
 * Alternativee implementation of setMotorSpeeds used for accelerating Tracy
 * in the positive x-axis; allowing for testing of the accelerometer in the x-axis.
 * This method will be replaced once the RayCastVehicle implementation is switch to 
 * a RigidVehicle implementation.
 * @method setMotorVelocity
 */
Tracy.prototype.setMotorVelocity = function(pulseModularWidth)
{
    /* 400 represents the maximum speed that can be set on physical Tracy */
    var MAX_PHYSICAL_TRACY_SPEED = 0.65;
    var MAX_INPUT_VALUE = 400;
    var powerOutput = pulseModularWidth/ MAX_INPUT_VALUE;
    var velocity = (MAX_PHYSICAL_TRACY_SPEED * 10) * powerOutput;

    this.chassisBody.velocity.x = velocity;
}

/**
 * Emulates the line sensor on the Tracy by raycasting downards and checks
 * if it has intersected with any lines.
 * @method getLineSensorReading
 * @return {Object} Returns a sensorReading object which has a .outerLeft .innerLeft
 *                  .outerRight .innerRight .middleSensor properties corresponding to
 *                  the position of the sensor relative to Pololu Zumo 32u4. The value
 *                  of the reading represents the brightness of the body it intersects.
 */
Tracy.prototype.getLineSensorReading = function()
{
    /* Local coordinates of line sensors that are situated below Tracy */
    outerLeftSensorLocal = new CANNON.Vec3(0.225,0,0);
    innerLeftSensorLocal = new CANNON.Vec3(0.45,0,0);
    outerRightSensorLocal = new CANNON.Vec3(-0.225,0,0);
    innerRightSensorLocal = new CANNON.Vec3(0.45,0,0);
    middleSensorLocal = new CANNON.Vec3(0,0,0);

    // Global coordinates of sensor on Tracy
    worldSource = robots[0].chassisBody.pointToWorldFrame(localSource);
    worldTarget = robots[0].chassisBody.pointToWorldFrame(localTarget);
    outerLeftSensorResult = new CANNON.RaycastResult();
    world.rayTest(worldSource,worldTarget,outerLeftSensorResult);

    if(outerLeftSensorResult.hasHit)
    {
        alert(outerLeftSensorResult.body.brightness);
    }
}

/**
 * Emulates the proximity sensor on the Pololu Zumo 32u4 by raycasting in a 2D arc.
 * The arc has radius 4 and angle of 20 degrees. Any objects that the ray intersects
 * within this area will be considered as an detected object.
 * @method getProximitySensor
 * @return {Object} Returns a sensorReadings object that has .left .right .front properties
 *                  that can be accessed. The values represent the distance to the detected object.
 */
Tracy.prototype.getProximitySensorReading = function()
{
    /* Object used to store the sensor readings which
    represent the distance to detected object */
    var sensorReadings = 
    {
        left: 0,
        right: 0,
        front: 0
    }

    /* The radius of the arc */
    var sensorRange = 4;

    /* The angle of the arc */
    var sensorAngle = 20;
    var sensorHalfAngle = sensorAngle/2;

    /* Get world coordinates of Tracy's current position which will be the starting point of all rays cast */
    var source = robots[0].chassisBody.position;

    /* Contains the results of the sensor readings */
    var results = [];

    /* Sweeps in an arc to find if any objects are in proximity */
    var getSensorReading = function(startAngle,endAngle)
    {
        /* Loop from the starting angle to end angle of the arc, 1 degree per iteration */
        for(angle=startAngle; angle<=endAngle; angle++)
        {
            var x = sensorRange * Math.sin(THREE.Math.degToRad(angle));
            var y = sensorRange * Math.cos(THREE.Math.degToRad(angle));
            var z = 0; /* 2D arc so we set z value to 0 */

            /* Convert local coordinates to world coordinates to cast ray */
            localTarget = new CANNON.Vec3(x,y,z);
            worldTarget = robots[0].chassisBody.pointToWorldFrame(localTarget);

            /* Cast ray and check result */
            sensorResult = new CANNON.RaycastResult();
            world.rayTest(source,worldTarget,sensorResult);

            if(sensorResult.hasHit) { return sensorResult.distance; }
        }

        return 0;
    }

    /* Create an arc in each of the front, right and left directions such that the arc is centered
    around the angles 0(front) 90(right) and 270(left) */
    sensorReadings.front = getSensorReading(0-sensorHalfAngle,0+sensorHalfAngle);
    sensorReadings.right = getSensorReading(90-sensorHalfAngle,90+sensorHalfAngle);
    sensorReadings.left = getSensorReading(270-sensorHalfAngle,270+sensorHalfAngle);

    return sensorReadings;
}


