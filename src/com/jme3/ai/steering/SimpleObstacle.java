/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering;

import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.math.Vector3f;

/**
 *
 * @author Brent Owens
 */
public class SimpleObstacle implements Obstacle {

    public Vector3f location;
    public float radius;
    public Vector3f velocity;

    public SimpleObstacle() {
        location = new Vector3f();
        velocity = new Vector3f();
    }

    public SimpleObstacle(Vector3f location, float radius, Vector3f velocity) {
        this.location = location;
        this.radius = radius;
        this.velocity = velocity;
    }

    @Override
    public Vector3f getVelocity() {
        return velocity;
    }

    @Override
    public Vector3f getLocation() {
        return location;
    }

    @Override
    public float getRadius() {
        return radius;
    }

    void update(Vector3f location, float radius, Vector3f velocity) {
        this.location.set(location);
        this.radius = radius;
        this.velocity.set(velocity);

    }
}
