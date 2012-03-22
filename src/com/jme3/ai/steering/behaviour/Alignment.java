/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.behaviour;

import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.math.Vector3f;
import java.util.List;

/**
 *
 * @author Shirkit
 */
public class Alignment implements Behaviour {

    public Vector3f calculateForce(Vector3f velocity, List<Obstacle> neighbors) {

        if (neighbors.isEmpty()) {
            return Vector3f.ZERO;
        }

        Vector3f alignment = new Vector3f();

        int count = 0;
        for (Obstacle v : neighbors) {
            if (!v.getVelocity().equals(Vector3f.ZERO)) {
                alignment.addLocal(v.getVelocity());
                count++;
            }
        }

        if (count > 0) {
            alignment.divideLocal(neighbors.size());
        }

        return alignment.subtract(velocity);
    }
}
