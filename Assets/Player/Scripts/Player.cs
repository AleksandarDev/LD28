using System;
using System.Security.Cryptography.X509Certificates;
using Assets.Managers.GameEvents;
using Assets.Managers.Selection;
using UnityEngine;
using System.Collections;

public class Player : SelectableGameObject {
	public float EngineForce = 2000f;
	public float ThrusterForce = 300f;

	public static Player Instance;
	public float MaxSpeed = 0.4f;
	public float Speed;
	public bool IsEngineWorking;

	private SmoothFollow cameraFollow;

	private PlayerStates state;
	private SolarSystemObject targetObject;
	private Vector3 lastPosition;


	public Player() {
		Player.Instance = this;
	}


	public void Start () {
		this.lastPosition = this.transform.position;
		this.cameraFollow = Camera.main.GetComponent<SmoothFollow>();
		this.FocusSelected();

		var engineKey = new GameEventsManager.WatchedKey(KeyCode.Space);
		engineKey.OnKeyDown += EngineKeyOnOnKey;
		engineKey.OnHold += EngineKeyOnOnKey;

		this.SetupFrontThrusters();
	}

	private void EngineKeyOnOnKey(GameEventsManager sender, GameEventsManager.WatchedKey source) {
		this.ApplyEngineForce();
	}

	private void ApplyEngineForce() {
		if (this.Speed < this.MaxSpeed) 
			this.rigidbody.AddForce(this.transform.forward * this.EngineForce * Time.deltaTime);
	}

	private void SetupFrontThrusters() {
		var frontLeft = new GameEventsManager.WatchedKey(KeyCode.Q);
		var frontRight = new GameEventsManager.WatchedKey(KeyCode.E);
		var frontUp = new GameEventsManager.WatchedKey(KeyCode.W);
		var frontDown = new GameEventsManager.WatchedKey(KeyCode.S);

		frontLeft.OnKeyDown += FrontLeftOnOnKey;
		frontLeft.OnHold += FrontLeftOnOnKey;

		frontRight.OnKeyDown += FrontRightOnOnKey;
		frontRight.OnHold += FrontRightOnOnKey;

		frontUp.OnKeyDown += FrontUpOnOnKey;
		frontUp.OnHold += FrontUpOnOnKey;

		frontDown.OnKeyDown += FrontDownOnOnKey;
		frontDown.OnHold += FrontDownOnOnKey;

		GameEventsManager.Instance.AddWatchedKey(frontLeft);
		GameEventsManager.Instance.AddWatchedKey(frontRight);
		GameEventsManager.Instance.AddWatchedKey(frontUp);
		GameEventsManager.Instance.AddWatchedKey(frontDown);
	}

	public void Update () {
		if (this.targetObject != null) {
			// TODO Animate
			// Add engine force if needed
			if (this.Speed < this.MaxSpeed) {
				this.ApplyEngineForce();
				this.IsEngineWorking = true;
			}
			else this.IsEngineWorking = false;

			// TODO Animate (add slowdown and changing course)
			//var direction = this.transform.position - this.targetObject.transform.position;
			//this.transform.Translate(direction.normalized*this.Speed*Time.deltaTime);
		}

		// TODO Enter orbit if close

		// Calculate speed
		this.Speed = Vector3.Distance(this.transform.position, this.lastPosition);
		this.lastPosition = this.transform.position;
	}


	public void TravelToOrbit(SolarSystemObject targetObject) {
		this.targetObject = targetObject;
		this.Speed = MaxSpeed;
		

		this.FocusTravel();
	}


	public void FocusSelected() {
		this.cameraFollow.distance = 1.6f;
		this.cameraFollow.height = 100f;
		this.cameraFollow.heightDamping = 4f;
		this.cameraFollow.rotationDamping = 3f;
		this.cameraFollow.target = this.transform;
	}

	public void FocusTravel() {
		this.cameraFollow.distance = 2f;
		this.cameraFollow.height = 0.3f;
		this.cameraFollow.heightDamping = 4f;
		this.cameraFollow.rotationDamping = 3f;
		this.cameraFollow.target = this.transform;
	}

	public override bool Selected(SelectionManager selectionManager) {
		this.FocusSelected();

		return base.Selected(selectionManager);
	}

	public override bool Deselected(SelectionManager selectionManager) {
		return false;
	}

	#region Front Thrusters

	private void FrontRightOnOnKey(GameEventsManager sender, GameEventsManager.WatchedKey source) {
		this.FrontThrustersHorizontal(true);
	}

	private void FrontLeftOnOnKey(GameEventsManager sender, GameEventsManager.WatchedKey source) {
		this.FrontThrustersHorizontal(false);
	}

	private void FrontUpOnOnKey(GameEventsManager sender, GameEventsManager.WatchedKey source) {
		this.FrontThrustersVertical(true);
	}

	private void FrontDownOnOnKey(GameEventsManager sender, GameEventsManager.WatchedKey source) {
		this.FrontThrustersVertical(false);
	}

	private void FrontThrustersVertical(bool toUp) {
		this.rigidbody.AddForceAtPosition(this.transform.up * (toUp ? 1 : -1) * this.ThrusterForce * Time.deltaTime, this.transform.position + this.transform.forward);
	}

	private void FrontThrustersHorizontal(bool toLeft) {
		this.rigidbody.AddForceAtPosition(this.transform.right * (toLeft ? -1 : 1) * this.ThrusterForce * Time.deltaTime, this.transform.position + this.transform.forward);
	}

	#endregion

	public enum PlayerStates {
		InOrbit,
		LeavingOrbit,
		InFlight,
		BeginningWarp,
		InWarp,
		EndingWarp,
		ChangingCourse,
		SynchronizingOrbit
	}
}
