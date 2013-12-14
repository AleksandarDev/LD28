using System;
using Assets.Managers.GameEvents;
using UnityEditor;
using UnityEngine;

namespace Assets.Gameplay.Camera {
	public class CameraControl : MonoBehaviour {
		public string ManagersContainerName = "Managers";


		public void Start () {
			// Get managers container
			var managersContainerObject = GameObject.Find(this.ManagersContainerName);
			if (managersContainerObject == null)
				throw new NullReferenceException("Managers container not found under name\"" + this.ManagersContainerName  + "\"");

			// Get used managers container components
			var gameEvents = managersContainerObject.GetComponent<GameEventsManager>();
			if (gameEvents == null) 
				throw new NullReferenceException("GameEventsManager not found in Managers container");

			GameEventsManager.WatchedKey moveLeftKey = new GameEventsManager.WatchedKey(KeyCode.LeftArrow);
			moveLeftKey.OnHold += MoveLeftKeyOnHold;
		}

		private void MoveLeftKeyOnHold(GameEventsManager sender, GameEventsManager.WatchedKey source) {
			this.transform.Translate(1 * Time.deltaTime, 0, 0);
		}

		public void Update () {
	
		}
	}
}
