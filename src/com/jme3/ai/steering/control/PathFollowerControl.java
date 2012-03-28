/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.control;

import com.jme3.ai.steering.behaviour.Alignment;
import com.jme3.ai.steering.behaviour.Cohesion;
import com.jme3.ai.steering.behaviour.Persuit;
import com.jme3.ai.steering.behaviour.Seek;
import com.jme3.ai.steering.behaviour.Separation;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.ai.steering.utilities.SteerControl;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Sphere;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Shirkit
 */
public class PathFollowerControl extends SimpleControl {

    private SteerControl outer;
    protected Persuit persuit = new Persuit();
    protected Seek seek = new Seek();
    protected Alignment alignment = new Alignment();
    protected Cohesion cohesion = new Cohesion();
    protected Separation separation = new Separation();
    protected float radius = 10f;
    private float minDistanceToWaypoint = 5f;
    boolean arrived;

    public PathFollowerControl(SteerControl outer) {
        this.outer = outer;
        arrived = false;
        Material mat = new Material(outer.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", ColorRGBA.Red);
        g.setMaterial(mat);
    }
    Geometry g = new Geometry("kkk", new Sphere(6, 6, 0.3f));

    @Override
    protected void controlUpdate(float tpf) {

        AbstractVehicle self = (AbstractVehicle) getSpatial();
        if (!self.getParent().hasChild(g))
            self.getParent().attachChild(g);

        List<Obstacle> neighbors = outer.neighboursNearby(self, radius);
        List<Obstacle> obstacles = new ArrayList<Obstacle>(neighbors);
        obstacles.addAll(outer.obstacalsNearby(self, radius));

        Vector3f seperationForce = this.separation.calculateForce(self.getWorldTranslation(), obstacles);
        Vector3f cohesionForce = this.cohesion.calculateForce(self.getWorldTranslation(), neighbors);
        Vector3f alignmentForce = this.alignment.calculateForce(self.velocity, neighbors);
        Vector3f momentumForce = self.velocity;
        Vector3f persuitForce = persuit.calculateForce(self.getWorldTranslation(), self.velocity, self.speed, 0f, tpf, Vector3f.ZERO, self.flock.getPath().getWaypoints().get(self.flock.lastPath).getPosition());

        cohesionForce.multLocal(0.5f);
        alignmentForce.multLocal(0.5f);
        seperationForce.multLocal(100f);
        momentumForce.multLocal(1f);
        persuitForce.multLocal(1f);

        Vector3f flock = seperationForce.add(cohesionForce).add(alignmentForce).add(momentumForce).add(persuitForce);
        try {
            g.setLocalTranslation(self.flock.getRelativePosition(self).add(self.flock.pivot.getWorldTranslation()));
            Vector3f seekForce = this.seek.calculateForce(self.getWorldTranslation(), self.velocity, self.speed, self.flock.getRelativePosition(self).add(self.flock.pivot.getWorldTranslation()));
            flock.subtractLocal(cohesionForce).subtractLocal(seperationForce).addLocal(seekForce.mult(1f));
        } catch (Exception e) {
        }

        minDistanceToWaypoint = self.collisionRadius * 5;

        if (minDistanceToWaypoint >= self.flock.getPath().getWaypoints().get(self.flock.lastPath).getPosition().distance(self.getWorldTranslation())) {
            self.flock.lastPath++;
            if (self.flock.lastPath >= self.flock.getPath().getWaypoints().size()) {
                arrived = true;
                self.flock.lastPath--;
                self.velocity = Vector3f.ZERO;
            }
        }

        self.updateVelocity(flock, tpf);
    }
}
