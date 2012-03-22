/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.control;

import com.jme3.ai.steering.behaviour.Flee;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.math.Vector3f;

/**
 *
 * @author Shirkit
 */
public class FleeControl extends SimpleControl {

    private Flee flee = new Flee();
    private AbstractVehicle target;

    public FleeControl(AbstractVehicle target) {
        this.target = target;
    }

    @Override
    protected void controlUpdate(float tpf) {
        AbstractVehicle vehicle = (AbstractVehicle) getSpatial();
        if (vehicle == null) {
            return;
        }
        // calculate the steering force from the Flee routine
        Vector3f steering = flee.calculateForce(vehicle.getWorldTranslation(), vehicle.velocity, vehicle.speed, target.getWorldTranslation());
        // add the force to the velicity
        vehicle.updateVelocity(steering, tpf);
    }
}
