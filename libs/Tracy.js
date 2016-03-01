function Simulator(x,y,z)
{
    var chassisShape = new CANNON.Box(new CANNON.Vec3(2.5, 2.5,0.5));
    var chassisBody = new CANNON.Body({ mass: 150 });
    chassisBody.addShape(chassisShape);
    chassisBody.position.set(x,y,z);
    chassisBody.angularVelocity.set(0, 0, 0.7);
    demo.addVisual(chassisBody);

    // Settings for the vehicle.
    var options = {
        radius: 1.4,
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

    // Create the vehicle
    var vehicle = new CANNON.RaycastVehicle({
        chassisBody: chassisBody,
    });

    options.chassisConnectionPointLocal.set(1.5, 2.5, 0);
    vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(1.5, -2.5, 0);
    vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(-1.5, 2.5, 0);
    vehicle.addWheel(options);

    options.chassisConnectionPointLocal.set(-1.5, -2.5, 0);
    vehicle.addWheel(options);

    vehicle.addToWorld(world);

    // Create wheel shapes.
    vehicle.wheelBodies = [];
    for(var i=0; i<vehicle.wheelInfos.length; i++){
        var wheel = vehicle.wheelInfos[i];
        var cylinderShape = new CANNON.Cylinder(wheel.radius, wheel.radius, wheel.radius / 2, 20);
        var wheelBody = new CANNON.Body({
            mass: 0
        });
        wheelBody.type = CANNON.Body.KINEMATIC;
        wheelBody.collisionFilterGroup = 0; // turn off collisions
        var q = new CANNON.Quaternion();
        q.setFromAxisAngle(new CANNON.Vec3(1, 0, 0), Math.PI / 2);
        wheelBody.addShape(cylinderShape, new CANNON.Vec3(), q);
        vehicle.wheelBodies.push(wheelBody);
        demo.addVisual(wheelBody);
        world.addBody(wheelBody);
    }

    vehicles.push(vehicle);
}

function handler(event){
            var up = (event.type == 'keyup');

            if(!up && event.type !== 'keydown'){
                return;
            }

            // Set breaks to reset car to initial position.
            vehicles[0].setBrake(0, 0);
            vehicles[0].setBrake(0, 1);
            vehicles[0].setBrake(0, 2);
            vehicles[0].setBrake(0, 3);

            switch(event.keyCode){
            case 38: // forward
                vehicles[0].applyEngineForce(up ? 0 : -maxForce*boost, 2);
                vehicles[0].applyEngineForce(up ? 0 : -maxForce*boost, 3);
                break;

            case 40: // backward
                vehicles[0].applyEngineForce(up ? 0 : maxForce*boost, 2);
                vehicles[0].applyEngineForce(up ? 0 : maxForce*boost, 3);
                break;

            case 66: // b
                vehicles[0].setBrake(brakeForce, 0);
                vehicles[0].setBrake(brakeForce, 1);
                vehicles[0].setBrake(brakeForce, 2);
                vehicles[0].setBrake(brakeForce, 3);
                break;

            case 70: // f
                  useSensors();
                  break;

            case 39: // right
                vehicles[0].applyEngineForce(up ? 0 : rotationForce, 1);
                vehicles[0].applyEngineForce(up ? 0 : rotationForce, 3);
                vehicles[0].applyEngineForce(up ? 0 : -rotationForce, 0);
                vehicles[0].applyEngineForce(up ? 0 : -rotationForce, 2);
                break;

            case 37: // left
                vehicles[0].applyEngineForce(up ? 0 : -rotationForce, 1);
                vehicles[0].applyEngineForce(up ? 0 : -rotationForce, 3);
                vehicles[0].applyEngineForce(up ? 0 : rotationForce, 0);
                vehicles[0].applyEngineForce(up ? 0 : rotationForce, 2);
                break;

            }
        }