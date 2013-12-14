using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using System.Collections;
using Vectrosity;

public class Planet : SolarSystemObject {
	private const double Pi2 = Math.PI * 2.0;

	public float RotationSpeed = 0.01f;
	public Material TrajectorTrailMaterial;
	public float TrajectoryTrailDetail = 0.1f;
	public float TrajectoryTrailSize = 1f;

	private VectorLine trajectoryPath;
	private Vector3 rotationAxis = new Vector3(0f, 1f, 0f);
	private float distanceFromSun;

	// GUI
	private TextMesh planetName;


	public override void Awake() {
		base.Awake();

		var guiTextElements = this.GetComponentsInChildren<TextMesh>();
		this.planetName = guiTextElements.FirstOrDefault(gte => gte.name == "PlanetName");

		this.planetName.text = "Earth";
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

		if (this.IsSelected)
			this.trajectoryPath.Draw3D(this.transform);

		this.planetName.transform.RotateAround(this.transform.position, new Vector3(0, 1, 0), 1f);
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
}
