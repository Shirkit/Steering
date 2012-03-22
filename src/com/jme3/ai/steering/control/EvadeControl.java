/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.control;

import com.jme3.ai.steering.behaviour.Evade;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.math.Vector3f;

/**
 *
 * @author Shirkit
 */
public class EvadeControl extends SimpleControl {

    private Evade evade = new Evade();
    private AbstractVehicle target;

    public EvadeControl(AbstractVehicle target) {
        this.target = target;
    }

    @Override
    protected void controlUpdate(float tpf) {
        AbstractVehicle vehicle = (AbstractVehicle) getSpatial();
        if (vehicle == null) {
            return;
        }
        // calculate the steering force from the Persuit routine
        Vector3f steering = evade.calculateForce(vehicle.getWorldTranslation(), vehicle.velocity, vehicle.speed, target.speed, tpf, target.velocity, target.getFuturePosition(tpf));
        // add the force to the velicity
        vehicle.updateVelocity(steering, tpf);
    }
}
