package com.jme3.ai.steering.utilities;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;

/**
 *
 * @author Shirkit
 */
public abstract class AbstractVehicle extends Node {

    public float speed = 1.5f; // worldUnits/second
    public float maxSpeed = 1.5f; // worldUnits/second
    public float maxTurnForce = 1f; // max steering force per second (perpendicular to velocity)
    // if speed is 1 and turn force is 1, then it will turn 45 degrees in a second
    public float mass = 1.0f; // the higher, the slower it turns
    public float collisionRadius = 0.4f;
    public Vector3f velocity = Vector3f.ZERO;
    public Flock<AbstractVehicle> flock;
    // debug geoms
    protected Geometry collisionLine;
    protected Geometry velocityLine;

    /**
     * Take the steering influence and apply the vehicle's mass, max speed,
     * speed, and maxTurnForce to determine the new velocity.
     */
    public void updateVelocity(Vector3f steeringInfluence, float scale) {
        Vector3f steeringForce = truncate(steeringInfluence, maxTurnForce * scale);
        Vector3f acceleration = steeringForce.divide(mass);
        Vector3f vel = truncate(velocity.add(acceleration), maxSpeed);
        velocity = vel;
        velocity.y = 0;

        setLocalTranslation(getLocalTranslation().add(velocity.mult(scale)));

        // rotate to face
        Quaternion rotTo = getLocalRotation().clone();
        rotTo.lookAt(velocity.normalize(), Vector3f.UNIT_Y);

        setLocalRotation(rotTo);
    }

    /**
     * truncate the length of the vector to the given limit
     */
    private Vector3f truncate(Vector3f source, float limit) {
        if (source.lengthSquared() <= limit * limit) {
            return source;
        } else {
            return source.normalize().scaleAdd(limit, Vector3f.ZERO);
        }
    }

    /**
     * Gets the predicted position for this 'frame', 
     * taking into account current position and velocity.
     * @param tpf time per fram
     */
    public Vector3f getFuturePosition(float tpf) {
        return getWorldTranslation().add(velocity);
    }
}
