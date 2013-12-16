using System;
using System.Collections.Generic;
using UnityEngine;
using System.Collections;
using Vectrosity;

public class SolarSystem : MonoBehaviour {
	public Sun Sun;
	public List<SolarSystemObject> Objects;
	public Material TradingRouteMaterial;
	public Material GridMaterial;
	
	private VectorLine tradeRoutes;
	private float tradingRouteSize = 1f;
	private float tradeRoutesMaxLength = 2000f;


	public SolarSystem() {
		this.Objects = new List<SolarSystemObject>();
	}


	public void Awake() {
		
	}

	public void Start () {
		// Populate Objects list
		var childGameObjects = this.GetComponentsInChildren<SolarSystemObject>();
		foreach (var childObject in childGameObjects)
			if (!(childObject is Sun))
				this.Objects.Add(childObject);
	}
	
	public void Update () {
		VectorLine.Destroy(ref tradeRoutes);
		this.FillTradeRoutes();
		this.tradeRoutes.Draw3DAuto();
	}


	private void FillTradeRoutes() {
		var routePoints = new List<Vector3>();
		for (int n = 0; n < this.Objects.Count; n++) {
			var currentObject = this.Objects[n];
			for (int index = n; index < this.Objects.Count; index++) {
				if (currentObject.GetDistanceFrom(this.Objects[index]) < this.tradeRoutesMaxLength) {
					routePoints.Add(currentObject.transform.position);
					routePoints.Add(this.Objects[index].transform.position);
				}
			}
		}

		// Instantiate trading route path
		this.tradeRoutes = new VectorLine(
			"TradingRoutesPath",
			routePoints.ToArray(),
			this.TradingRouteMaterial,
			this.tradingRouteSize,
			LineType.Discrete);
	}
}
