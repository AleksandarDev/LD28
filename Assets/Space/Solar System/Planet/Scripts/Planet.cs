using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using System.Collections;
using Vectrosity;

// http://scrawkblog.com/2013/04/13/gpu-gems-to-unity-atmospheric-scattering/
// http://entitycrisis.blogspot.com/2011/01/unity3d-30-planet-shader.html
public class Planet : SolarSystemObject {
	private const double Pi2 = Math.PI * 2.0;

	public PlanetDetails Details;
	public float RotationSpeed = 0.01f;
	public Material TrajectorTrailMaterial;
	public float TrajectoryTrailDetail = 0.1f;
	public float TrajectoryTrailSize = 1f;

	private VectorLine trajectoryPath;
	private Vector3 rotationAxis = new Vector3(0f, 1f, 0f);
	private float distanceFromSun;


	public override void Awake() {
		base.Awake();
	}

	public override void Start() {
		base.Start();

		this.distanceFromSun = this.GetDistanceFrom(this.Sun);


		// TODO call this on distanceFromSun change
		// TODO Fix offset bug
		this.FillTrajectoryTrail();
	}

	public override void Update() {
		base.Update();

		// Circle rotation
		this.transform.RotateAround(
			this.Sun.transform.position,
			this.rotationAxis,
			this.RotationSpeed);

		if (this.IsSelected) {
			this.trajectoryPath.Draw3D(this.Sun.transform);
		}
	}

	public void FillTrajectoryTrail() {
		// Build points list
		var pointsList = new List<Vector3>();
		for (float theta = 0; theta < Pi2; theta += this.TrajectoryTrailDetail) {
			var x = (float) (this.distanceFromSun*Math.Cos(theta));
			var z = (float) (this.distanceFromSun*Math.Sin(theta));
			pointsList.Add(new Vector3(x, this.transform.position.y, z));
		}

		// Create path line
		this.trajectoryPath = new VectorLine(
			"PlanetTrajectoryPath",
			pointsList.ToArray(),
			this.TrajectorTrailMaterial,
			this.TrajectoryTrailSize, 
			LineType.Continuous);
	}

	[Serializable]
	public class PlanetDetails {
		public string Name;
		public char Type;
		public int Population;
		public float Speed;
		public string Resources;
	}
}
