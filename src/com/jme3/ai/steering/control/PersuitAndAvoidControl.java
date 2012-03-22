/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.control;

import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.ai.steering.behaviour.ObstacleAvoid;
import com.jme3.ai.steering.behaviour.Persuit;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.ai.steering.utilities.SteerControl;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Shirkit
 */
public class PersuitAndAvoidControl extends SimpleControl {

    private Persuit persuit = new Persuit();
    private ObstacleAvoid avoid = new ObstacleAvoid();
    private List<Obstacle> obstacles = new ArrayList<Obstacle>();
    private AbstractVehicle target;
    private SteerControl outer;

    public PersuitAndAvoidControl(SteerControl outer, AbstractVehicle target) {
        this.target = target;
        this.outer = outer;

    }

    @Override
    protected void controlUpdate(float tpf) {
        AbstractVehicle vehicle = (AbstractVehicle) getSpatial();
        if (vehicle == null) {
            return;
        }
        if (obstacles.isEmpty()) {
            obstacles.addAll(outer.getObstacals());
        }
        // calculate the steering force from the Persuit routine
        Vector3f steering = persuit.calculateForce(vehicle.getWorldTranslation(), vehicle.velocity, vehicle.speed, target.speed, tpf, target.velocity, target.getFuturePosition(tpf));
        Vector3f avoidance = avoid.calculateForce(vehicle.getWorldTranslation(), vehicle.velocity, vehicle.speed, vehicle.collisionRadius, vehicle.maxTurnForce, tpf, obstacles);
        // add the force to the velocity
        vehicle.updateVelocity(steering.add(avoidance), tpf);
    }
}
