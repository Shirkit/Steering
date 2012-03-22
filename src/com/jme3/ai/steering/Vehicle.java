package com.jme3.ai.steering;

import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Line;
import com.jme3.ai.steering.utilities.SteerControl;

/**
 * A Vehicle's "forward" is along the local +X axis
 */
class Vehicle extends AbstractVehicle {

    private SteerControl steerControl;

    public Vehicle(ColorRGBA color, SteerControl steerControl) {
        this.steerControl = steerControl;

        Box b = new Box(Vector3f.ZERO, 0.1f, 0.1f, 0.2f);
        Geometry geom = new Geometry("Box", b);
        Material mat = new Material(steerControl.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", color);
        geom.setMaterial(mat);
        this.attachChild(geom);

    }

    @Override
    public void updateVelocity(Vector3f steeringInfluence, float scale) {
        super.updateVelocity(steeringInfluence, scale);
        showVelocity(velocity.length(), true);
        //showCollisionRange(speed / maxTurnForce, true);
    }

    public void showVelocity(float length, boolean show) {
        if (velocityLine == null) {
            // create it if it doesn't exist
            //Vector3f end = this.velocity.normalize().mult(length);
            Line line = new Line(Vector3f.ZERO, new Vector3f(0, 0, length));
            Geometry geom = new Geometry("cylinder", line);
            Material mat = new Material(steerControl.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
            mat.setColor("Color", ColorRGBA.Cyan);
            geom.setMaterial(mat);
            velocityLine = geom;
        }

        // modify its direction and length
        Line line = (Line) velocityLine.getMesh();
        line.updatePoints(Vector3f.ZERO, new Vector3f(0, 0, length));

        // attach/detach it
        if (show) {
            if (velocityLine.getParent() == null) {
                this.attachChild(velocityLine);
            }
        } else {
            if (velocityLine.getParent() != null) {
                velocityLine.removeFromParent();
            }
        }
    }

    public void showCollisionRange(float length, boolean show) {
        if (collisionLine == null) {
            // create it if it doesn't exist
            //Vector3f end = this.velocity.normalize().mult(length);
            Line line = new Line(Vector3f.ZERO, new Vector3f(0, 0, length));
            Geometry geom = new Geometry("cylinder", line);
            Material mat = new Material(steerControl.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
            mat.setColor("Color", ColorRGBA.Pink);
            geom.setMaterial(mat);
            collisionLine = geom;
        }

        // modify its direction and length
        Line line = (Line) collisionLine.getMesh();
        line.updatePoints(Vector3f.ZERO, new Vector3f(0, 0, length));

        // attach/detach it
        if (show) {
            if (collisionLine.getParent() == null) {
                this.attachChild(collisionLine);
            }
        } else {
            if (collisionLine.getParent() != null) {
                collisionLine.removeFromParent();
            }
        }
    }

    public void setCollideWarning(boolean warn) {
        if (collisionLine != null) {
            if (warn) {
                collisionLine.getMaterial().setColor("Color", ColorRGBA.Red);
            } else {
                collisionLine.getMaterial().setColor("Color", ColorRGBA.Cyan);
            }
        }
    }
    protected SimpleObstacle me;

    public Obstacle toObstacle() {
        if (me == null) {
            me = new SimpleObstacle();
        }
        me.update(getWorldTranslation(), mass, velocity);
        return me;
    }
}
