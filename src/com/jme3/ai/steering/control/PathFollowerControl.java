/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.control;

import com.jme3.ai.steering.behaviour.Alignment;
import com.jme3.ai.steering.behaviour.Cohesion;
import com.jme3.ai.steering.behaviour.Persuit;
import com.jme3.ai.steering.behaviour.Separation;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.ai.steering.utilities.SteerControl;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import java.util.List;
import jme3tools.navmesh.Path;

/**
 *
 * @author Shirkit
 */
public class PathFollowerControl extends SimpleControl {

    private SteerControl outer;
    private Persuit persuit = new Persuit();
    protected Alignment alignment = new Alignment();
    protected Cohesion cohesion = new Cohesion();
    protected Separation separation = new Separation();
    protected float radius = 10f;
    private float minDistanceToWaypoint = 5f;
    boolean arrived;

    public PathFollowerControl(SteerControl outer) {
        this.outer = outer;
        arrived = false;
    }

    @Override
    protected void controlUpdate(float tpf) {

        AbstractVehicle self = (AbstractVehicle) getSpatial();

        List<Obstacle> neighbors = outer.neighboursNearby(self, radius);
        List<Obstacle> obstacles = new ArrayList<Obstacle>(neighbors);
        obstacles.addAll(outer.obstacalsNearby(self, radius));

        Vector3f seperationForce = this.separation.calculateForce(self.getWorldTranslation(), obstacles);
        Vector3f cohesionForce = this.cohesion.calculateForce(self.getWorldTranslation(), neighbors);
        Vector3f alignmentForce = this.alignment.calculateForce(self.velocity, neighbors);
        Vector3f momentumForce = self.velocity;
        Vector3f persuitForce = persuit.calculateForce(self.getWorldTranslation(), self.velocity, self.speed, 0f, tpf, Vector3f.ZERO, self.flock.path.getWaypoints().get(self.flock.lastPath).getPosition());

        cohesionForce.multLocal(0.5f);
        alignmentForce.multLocal(0.5f);
        seperationForce.multLocal(100f);
        momentumForce.multLocal(1f);
        persuitForce.multLocal(1f);

        Vector3f flock = seperationForce.add(cohesionForce).add(alignmentForce).add(momentumForce).add(persuitForce);

        minDistanceToWaypoint = self.collisionRadius * 5;

        if (minDistanceToWaypoint >= self.flock.path.getWaypoints().get(self.flock.lastPath).getPosition().distance(self.getWorldTranslation())) {
            self.flock.lastPath++;
            if (self.flock.lastPath >= self.flock.path.getWaypoints().size()) {
                arrived = true;
                self.flock.lastPath--;
                self.velocity = Vector3f.ZERO;
            }
        }

        self.updateVelocity(flock, tpf);
    }
}
