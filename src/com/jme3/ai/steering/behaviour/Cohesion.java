/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.behaviour;

import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.ai.steering.utilities.Flock;
import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.math.Vector3f;
import java.util.List;

/**
 *
 * @author Shirkit
 */
public class Cohesion implements Behaviour {

    public Vector3f calculateForce(Vector3f location, List<Obstacle> neighbors) {

        if (neighbors.isEmpty()) {
            return Vector3f.ZERO;
        }

        Vector3f cohesion = new Vector3f();

        int count = 0;
        for (Obstacle v : neighbors) {
            if (!v.getVelocity().equals(Vector3f.ZERO)) {
                count++;
                cohesion.addLocal(location.subtract(v.getLocation()));
            }
        }

        if (count > 0) {
            cohesion.divideLocal(count);
        }

        return cohesion.negate();
    }
}
