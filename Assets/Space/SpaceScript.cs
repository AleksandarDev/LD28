using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.Collections;

public class SpaceScript : MonoBehaviour {
	public Material GridMaterial;
	public List<SolarSystem> SolarSystems;

	private SpaceGrid grid;


	public SpaceScript() {
		this.SolarSystems = new List<SolarSystem>();
	}

	public void Start () {
		this.SolarSystems.AddRange(this.GetComponentsInChildren<SolarSystem>());
		this.grid = new SpaceGrid(this.GridMaterial);
		this.grid.ActiveResolution = SpaceGrid.AvailableResolutions.Planet;
	}
	
	public void Update () {
		this.grid.Update(this.SolarSystems.First().Objects.First().gameObject);
	}
}
