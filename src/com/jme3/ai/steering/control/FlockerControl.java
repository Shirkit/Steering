package com.jme3.ai.steering.control;

import com.jme3.ai.steering.behaviour.Alignment;
import com.jme3.ai.steering.behaviour.Cohesion;
import com.jme3.ai.steering.behaviour.Separation;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.ai.steering.utilities.SteerControl;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Shirkit
 */
public class FlockerControl extends SimpleControl {

    protected SteerControl outer;
    protected Alignment alignment = new Alignment();
    protected Cohesion cohesion = new Cohesion();
    protected Separation separation = new Separation();
    protected float radius = 10f;

    public FlockerControl(SteerControl outer) {
        this.outer = outer;
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
        
        cohesionForce.multLocal(1f);
        alignmentForce.multLocal(1f);
        seperationForce.multLocal(400f);
        momentumForce.multLocal(1f);
        
        Vector3f last = seperationForce.add(cohesionForce).add(alignmentForce).add(momentumForce);

        self.updateVelocity(last, tpf);
    }

    public Vector3f randomness() {
        float x = FastMath.nextRandomFloat() * 2f - 1.0f;
        float y = FastMath.nextRandomFloat() * 2f - 1.0f;
        float l = FastMath.sqrt(x * x + y * y);
        return new Vector3f(0.05f * x / l, 0f, 0.05f * y / l);
    }
}