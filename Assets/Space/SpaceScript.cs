using UnityEngine;
using System.Collections;

public class SpaceScript : MonoBehaviour {
	public Material GridMaterial;

	private SpaceGrid grid;


	public void Start () {
		this.grid = new SpaceGrid(this.GridMaterial);
		this.grid.ActiveResolution = SpaceGrid.AvailableResolutions.SolarSystem;
	}
	
	public void Update () {
		this.grid.Update();
	}
}
