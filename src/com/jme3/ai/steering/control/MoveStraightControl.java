/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.control;

import com.jme3.ai.steering.behaviour.Randomness;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.math.Vector3f;

/**
 *
 * @author Shirkit
 */
public class MoveStraightControl extends SimpleControl {

    protected Randomness randomness = new Randomness();

    public MoveStraightControl() {
    }

    @Override
    protected void controlUpdate(float tpf) {
        AbstractVehicle vehicle = (AbstractVehicle) getSpatial();
        if (vehicle == null) {
            return;
        }

        vehicle.updateVelocity(Vector3f.ZERO, tpf);
    }
}
