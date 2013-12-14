using System;
using UnityEditor;
using UnityEngine;
using System.Collections;

public class Planet : SolarSystemObject {
	private const double Pi2 = Math.PI * 2.0;

	public float RotationSpeed = 0.01f;
	public float TrajectoryTrailDetail = 0.1f;

	private Vector3 rotationAxis = new Vector3(0f, 1f, 0f);

	public virtual void Start() {
		
	}

	public virtual void Update() {
		// Circle rotation
		this.transform.RotateAround(
			this.Sun.transform.position,
			this.rotationAxis,
			this.RotationSpeed);

		this.DrawTrajectoryTrail();
	}

	public void DrawTrajectoryTrail() {
		// Total number of points in circle.
		int totalPoints = (int) ((2.0*Math.PI)/this.TrajectoryTrailDetail);
		this.LineRenderer.SetVertexCount(totalPoints);

		float
			x = this.transform.position.x,
			z = this.transform.position.z;
		float radius = this.GetDistanceFrom(this.Sun);

		int index = 0;
		for (float theta = 0; theta < Pi2 && index < totalPoints; theta += this.TrajectoryTrailDetail) {
			x = (float)(radius * Math.Cos(theta));
			z = (float)(radius * Math.Sin(theta));

			this.LineRenderer.SetPosition(
				index++,
				new Vector3(x, this.transform.position.y, z));
		}
	}


	public float GetDistanceFrom(SolarSystemObject obj) {
		return Vector3.Distance(obj.transform.position, this.transform.position);
	}

	#region LineRenderer

	private LineRenderer lineRenderer;

	public LineRenderer LineRenderer {
		get {
			if (this.lineRenderer == null)
				this.lineRenderer = this.GetComponentInChildren<LineRenderer>();
			return this.lineRenderer;
		}
	}

	#endregion
}
