/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.control;

import com.jme3.ai.steering.behaviour.Seek;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.math.Vector3f;

/**
 *
 * @author Shirkit
 */
public class SeekControl extends SimpleControl {

    private Seek seek = new Seek();
    private AbstractVehicle target;

    public SeekControl(AbstractVehicle target) {
        this.target = target;
    }

    @Override
    protected void controlUpdate(float tpf) {
        AbstractVehicle vehicle = (AbstractVehicle) getSpatial();
        if (vehicle == null) {
            return;
        }
        // calculate the steering force from the Seek routine
        Vector3f steering = seek.calculateForce(vehicle.getWorldTranslation(), vehicle.velocity, vehicle.speed, target.getWorldTranslation());
        // add the force to the velicity
        vehicle.updateVelocity(steering, tpf);
    }
}
