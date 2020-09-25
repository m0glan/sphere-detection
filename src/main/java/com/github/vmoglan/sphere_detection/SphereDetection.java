package com.github.vmoglan.sphere_detection;

import com.github.vmoglan.pcljava.Point3d;
import com.github.vmoglan.pcljava.PointCloud3d;
import com.github.vmoglan.pcljava.RandomSampleConsensus;
import com.github.vmoglan.pcljava.SampleConsensusModelSphere;
import com.github.vmoglan.pcljava.visualization.Visualizer;
import com.github.vmoglan.pcljava.visualization.Visualizer3d;

public class SphereDetection {
	static {
		System.loadLibrary("pcljava");	// loading the native pcljava library	
	}
	
	public static void main(String[] args) {
		PointCloud3d noisySphere = generateNoisySphere(500);

		SampleConsensusModelSphere sphereModel = new SampleConsensusModelSphere(noisySphere);
		sphereModel.create();
		
		RandomSampleConsensus ransac = new RandomSampleConsensus(sphereModel);
		ransac.create();
		ransac.setDistanceThreshold(.01f);
		ransac.computeModel(0);
		PointCloud3d detectedSphere = ransac.getInliers(noisySphere);
		
		/*
		 * Color points belonging to the sphere in a different color to distinguish them 
		 * from noise.
		 */
		for (Point3d point : detectedSphere) {
			point.setRGB((short) 255, (short) 0, (short) 0);
		}
		

		// Initialize visualizer.
		Visualizer<Point3d> visualizer = new Visualizer3d();
		visualizer.create();
		visualizer.setWindowName("Sphere Detection");
		visualizer.setBackgroundColor(0.f, 0.f, 0.f);
		visualizer.addCoordinateSystem(0.2, "axis", 0);
		visualizer.addPointCloud(noisySphere, "noisySphere", 0);
		visualizer.addPointCloud(detectedSphere, "detectedSphere", 0);
		visualizer.setPointSize(2, "noisySphere");
		visualizer.setPointSize(3, "detectedSphere");
		
		// Run main loop.
		while (!visualizer.wasStopped()) {
			visualizer.spinOnce(100, false);
			
			try {
				Thread.sleep(100);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		
		// Free all memory allocated through native code.
		visualizer.dispose();
		detectedSphere.dispose();
		ransac.dispose();
		sphereModel.dispose();
		noisySphere.dispose();
	}
	
	/**
	 * @return a point cloud globally shaped as a sphere with certain points being
	 * outside of it.
	 */
	private static PointCloud3d generateNoisySphere(int pointCount) {
		PointCloud3d noisySphere = new PointCloud3d();
		noisySphere.create();
		
		for (int i = 0; i < pointCount; i++) {
			Point3d point = generatePoint(i);
			noisySphere.add(point);
			
			/*
			 * The reference to point created within the loop is not the same as the reference
			 * to the point within the noisySphere point cloud; as such it needs to be released
			 * at the end of each loop iteration.
			 */
			point.dispose(); 
		}
		
		return noisySphere;
	}

	private static Point3d generatePoint(int i) {
		Point3d point = new Point3d();
		point.create();

		float x = randomFloat(-1f, 1f);
		point.setX(x);

		float y = randomFloat(-1f, 1f);
		point.setY(y);
		
		float z;

		if (i % 5 == 0) {
			z = randomFloat(-1f, 1f);
		} else if (i % 2 == 0) {
			z = (float)Math.sqrt(1 - (point.getX() * point.getX()) - (point.getY() * point.getY()));
		} else {
			z = (float)-Math.sqrt(1 - (point.getX() * point.getX()) - (point.getY() * point.getY()));
		}

		point.setZ(z);
		
		point.setRGB((short)255, (short)255, (short)255);

		return point;
	}
	
	private static float randomFloat(float min, float  max) {
		return (float) (min + Math.random() * (max - min));
	}
}
