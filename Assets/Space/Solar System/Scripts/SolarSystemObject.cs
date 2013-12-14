using Assets.Managers.Selection;
using UnityEngine;
using System.Collections;

public class SolarSystemObject : SelectableGameObject {
	private SolarSystem solarSystem;


	public virtual void Awake() {
		// Retrieve SolarSystem component from parent GameObject
		var parentTransform = this.transform.parent;
		var parentGameObject = parentTransform.gameObject;
		this.solarSystem = parentGameObject.GetComponent<SolarSystem>();
	}

	public virtual void Start() {

	}

	public virtual void Update() {

	}

	public virtual float GetDistanceFrom(SolarSystemObject obj) {
		return Vector3.Distance(obj.transform.position, this.transform.position);
	}

	public Sun Sun {
		get { return this.solarSystem.Sun; }
	}
}
