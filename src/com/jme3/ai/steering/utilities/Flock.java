/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.utilities;

import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import java.util.ArrayList;
import java.util.HashMap;
import jme3tools.navmesh.Path;

/**
 *
 * @author Shirkit
 */
public class Flock<T extends AbstractVehicle> extends Node {

    public int lastPath;
    public T pivot;
    private Path path;
    public ArrayList<T> members;
    private HashMap<T, Vector3f> positions;

    public Flock() {
        lastPath = 0;
        members = new ArrayList<T>();
        positions = new HashMap<T, Vector3f>();
    }

    public boolean add(T e) {
        attachChild(e);
        return members.add(e);
    }

    public boolean has(T e) {
        return members.contains(e);
    }

    public boolean remove(T e) {
        detachChild(e);
        return members.remove(e);
    }

    public void setPath(Path p) {
        this.path = p;
    }

    public Path getPath() {
        return path;
    }

    public void registerFormation() {
        positions.clear();
        Vector3f center = new Vector3f();
        for (T e : members) {
            Vector3f pos = e.getLocalTranslation();
            center.addLocal(pos);
        }
        center.divideLocal(members.size());
        pivot = members.get(0);
        for (T e : members)
            if (e.getLocalTranslation().distance(center) < pivot.getLocalTranslation().distance(center))
                pivot = e;

        for (T e : members)
            positions.put(e, e.getLocalTranslation().subtract(pivot.getLocalTranslation()));
    }

    public Vector3f getRelativePosition(T e) {
        return positions.get(e);
    }
}
