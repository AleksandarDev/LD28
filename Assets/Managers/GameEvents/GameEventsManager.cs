using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.Managers.GameEvents {
	public delegate void MouseScrollEventHandler(GameEventsManager sender);

	public class GameEventsManager : MonoBehaviour, IManager {
		public DebugOptions Debug = new DebugOptions();

		// Mouse settings
		public MouseOnClickTypes OnClickType = MouseOnClickTypes.OnUp;
		public float DoubleClickInterval = 400f;

		// Keyboard settings
		public KeyboardOnKeyTypes OnKeyType = KeyboardOnKeyTypes.Released;
		public float HoldDelayInterval = 1000;
		public List<WatchedKey> WatchedKeys = new List<WatchedKey>(); 

		// Mouse click events
		public event MouseClickEventHandler OnLeftDoubleClick;
		public event MouseClickEventHandler OnLeftClick;
		public event MouseClickEventHandler OnLeftDown;
		public event MouseClickEventHandler OnLeftUp;
		public event MouseClickEventHandler OnRightDoubleClick;
		public event MouseClickEventHandler OnRightClick;
		public event MouseClickEventHandler OnRightDown;
		public event MouseClickEventHandler OnRightUp;

		// Mouse move events
		public event MouseMoveEventHandler OnMove;
		public event MouseMoveEventHandler OnMoveStarted;
		public event MouseMoveEventHandler OnMoveStopped;

		// Mouse scroll events
		public event MouseScrollEventHandler OnScroll;
		public event MouseScrollEventHandler OnScrollStarted;
		public event MouseScrollEventHandler OnScrollEnded;

		// Keyboard events
		public event KeyboardEventHandler OnAnyWatchedKey;
		public event KeyboardEventHandler OnAnyWatchedKeyHold;
		public event KeyboardEventHandler OnAnyWatchedKeyPressed;
		public event KeyboardEventHandler OnAnyWatchedKeyReleased;

		// Mouse click variables
		private const int WatchedMouseButtons = 2;
		private bool[] mouseState;
		private bool[] mouseStatePrevious;
		private bool[] mouseClickTimerEnabled;
		private float[] mouseClickTimer;

		// Mouse move variables
		private Vector3 mousePosition;
		private Vector3 mousePositionPrevious;
		private bool isMoving;


		private void Awake() { }

		private void Start() {
			// Mouse click
			this.mouseState = new bool[WatchedMouseButtons];
			this.mouseStatePrevious = new bool[WatchedMouseButtons];
			this.mouseClickTimerEnabled = new bool[WatchedMouseButtons];
			this.mouseClickTimer = new float[WatchedMouseButtons];

			// Register keys added in editor
			foreach (var watchedKey in this.WatchedKeys)
				this.RegisterWatchedKey(watchedKey);
		}

		private void Update() {
			this.UpdateMouseClick();
			this.UpdateMouseMove();
			this.UpdateKeyboard();
		}

		#region Mouse move

		private void UpdateMouseMove() {
			this.mousePositionPrevious = this.mousePosition;
			this.mousePosition = Input.mousePosition;

			if (this.mousePosition != this.mousePositionPrevious) {
				if (this.OnMove != null)
					this.OnMove(this, this.mousePosition);
				if (this.Debug.MouseMove)
					UnityEngine.Debug.Log(String.Format("MouseMove: new position {0}", this.mousePosition));

				if (!this.isMoving) {
					this.isMoving = true;
					if (this.OnMoveStarted != null)
						this.OnMoveStarted(this, this.mousePosition);

					if (this.Debug.MouseMove)
						UnityEngine.Debug.Log(String.Format("MoveStarted: new position {0}", this.mousePosition));
				}
			}
			else {
				if (isMoving) {
					this.isMoving = false;
					if (this.OnMoveStopped != null)
						this.OnMoveStopped(this, this.mousePosition);

					if (this.Debug.MouseMove)
						UnityEngine.Debug.Log(String.Format("MoveStopped: new position {0}", this.mousePosition));
				}
			}
		}

		#endregion

		#region Keyboard

		private void UpdateKeyboard() {
			foreach (var watchedKey in this.WatchedKeys)
				watchedKey.Update();
		}

		public void AddWatchedKey(KeyCode keyCode) {
			this.AddWatchedKey(new WatchedKey(keyCode));
		}

		public void AddWatchedKey(WatchedKey watchedKey) {
			this.WatchedKeys.Add(watchedKey);
			this.RegisterWatchedKey(watchedKey);
		}

		public void RemoveWatchedKey(KeyCode keyCode) {
			this.RemoveWatchedKey(
				this.WatchedKeys.FirstOrDefault(
					wk => wk.KeyCode == keyCode));
		}

		public void RemoveWatchedKey(WatchedKey watchedKey) {
			this.WatchedKeys.Remove(watchedKey);
			this.UnregisterWatchedKey(watchedKey);
		}

		public void RegisterWatchedKey(WatchedKey watchedKey) {
			watchedKey.OnKey += this.WatchedKeyOnKey;
			watchedKey.OnPressed += this.WatchedKeyOnPressed;
			watchedKey.OnReleased += this.WatchedKeyOnReleased;
			watchedKey.OnHold += this.WatchedKeyOnHold;
		}

		public void UnregisterWatchedKey(WatchedKey watchedKey) {
			watchedKey.OnKey -= this.WatchedKeyOnKey;
			watchedKey.OnPressed -= this.WatchedKeyOnPressed;
			watchedKey.OnReleased -= this.WatchedKeyOnReleased;
			watchedKey.OnHold -= this.WatchedKeyOnHold;
		}

		private void WatchedKeyOnKey(GameEventsManager sender, WatchedKey watchedKey) {
			if (this.OnAnyWatchedKey != null)
				this.OnAnyWatchedKey(this, watchedKey);

			if (this.Debug.Keyboard)
				UnityEngine.Debug.Log(String.Format("Keyboard: key {0}", watchedKey.KeyCode));
		}

		private void WatchedKeyOnPressed(GameEventsManager sender, WatchedKey watchedKey) {
			if (this.OnAnyWatchedKeyPressed != null)
				this.OnAnyWatchedKeyPressed(this, watchedKey);

			if (this.Debug.Keyboard)
				UnityEngine.Debug.Log(String.Format("Keyboard: pressed {0}", watchedKey.KeyCode));
		}

		private void WatchedKeyOnReleased(GameEventsManager sender, WatchedKey watchedKey) {
			if (this.OnAnyWatchedKeyReleased != null)
				this.OnAnyWatchedKeyReleased(this, watchedKey);

			if (this.Debug.Keyboard)
				UnityEngine.Debug.Log(String.Format("Keyboard: released {0}", watchedKey.KeyCode));
		}

		private void WatchedKeyOnHold(GameEventsManager sender, WatchedKey watchedKey) {
			if (this.OnAnyWatchedKeyHold != null)
				this.OnAnyWatchedKeyHold(this, watchedKey);

			if (this.Debug.Keyboard)
				UnityEngine.Debug.Log(String.Format("Keyboard: hold {0}", watchedKey.KeyCode));
		}

		[Serializable]
		public class WatchedKey {
			public GameEventsManager Manager;
			public KeyCode KeyCode;
			public bool IsDown;
			public bool IsHold;

			public event KeyboardEventHandler OnKey;
			public event KeyboardEventHandler OnHold;
			public event KeyboardEventHandler OnPressed;
			public event KeyboardEventHandler OnReleased;
			public event KeyboardEventHandler OnKeyDown;

			private bool isReleased;
			private float downTimer;



			public WatchedKey(KeyCode keyCode) {
				this.KeyCode = keyCode;
			}


			public void Update() {
				// Key pressed/released handling
				if (!this.IsDown) {
					this.IsDown = Input.GetKeyDown(this.KeyCode);
					if (this.IsDown && this.isReleased) {
						this.isReleased = false;
						if (this.OnPressed != null)
							this.OnPressed(this.Manager, this);

						if (this.Manager.OnKeyType == KeyboardOnKeyTypes.Pressed && 
							this.OnKey != null)
							this.OnKey(this.Manager, this);
					}
				}
				else {
					this.isReleased = Input.GetKeyUp(this.KeyCode);
					if (this.isReleased) {
						this.IsDown = this.IsHold = false;
						this.downTimer = 0;
						if (this.OnReleased != null)
							this.OnReleased(this.Manager, this);

						if (this.Manager.OnKeyType == KeyboardOnKeyTypes.Released &&
						    this.OnKey != null)
							this.OnKey(this.Manager, this);
					}
				}

				// Key hold handling
				if (this.IsDown) {
					this.downTimer += Time.deltaTime*1000;

					if (this.OnKeyDown != null)
						this.OnKeyDown(this.Manager, this);
				}
				if (this.downTimer >= this.Manager.HoldDelayInterval) {
					this.IsHold = true;
					if (this.OnHold != null)
						this.OnHold(this.Manager, this);
				}
			}
		}

		public enum KeyboardOnKeyTypes {
			Released,
			Pressed
		}

		#endregion

		#region Mouse click

		private void UpdateMouseClick() {
			// Update all mouse buttons
			for (int index = 0; index < WatchedMouseButtons; index++)
				this.UpdateMouseButton((MouseButtons) index);
		}

		private void UpdateMouseButton(MouseButtons button) {
			var buttonIndex = (int) button;

			// Update states
			this.mouseStatePrevious[buttonIndex] = this.mouseState[buttonIndex];
			this.mouseState[buttonIndex] = Input.GetMouseButton(buttonIndex);

			// Check if mouse button state changed
			if (this.mouseState[buttonIndex] != this.mouseStatePrevious[buttonIndex]) {
				// On left down/up
				if (this.mouseState[buttonIndex]) {
					this.OnMouseDown(button);
					if (this.OnClickType == MouseOnClickTypes.OnDown) {
						this.OnMouseClick(button);
					}
				}
				else {
					this.OnMouseUp(button);
					if (this.OnClickType == MouseOnClickTypes.OnUp) {
						this.OnMouseClick(button);
					}
				}
			}

			// Double click handling
			if (this.mouseClickTimerEnabled[buttonIndex]) {
				this.mouseClickTimer[buttonIndex] += Time.deltaTime * 1000f;
				if (this.mouseClickTimer[buttonIndex] >= this.DoubleClickInterval) {
					this.mouseClickTimerEnabled[buttonIndex] = false;
					this.mouseClickTimer[buttonIndex] = 0;
				}
			}
		}

		private void OnMouseDown(MouseButtons button) {
			if (button == MouseButtons.Left) {
				if (this.OnLeftDown != null)
					this.OnLeftDown(this, new MouseClickEventArgs(MouseButtons.Left, true, false, false));
			}
			else if (button == MouseButtons.Right) {
				if (this.OnRightDown != null)
					this.OnRightDown(this, new MouseClickEventArgs(MouseButtons.Right, true, false, false));
			}

			if (this.Debug.MouseClick)
				UnityEngine.Debug.Log(String.Format("MouseClick: down {0} at {1}", button, this.mousePosition));
		}

		private void OnMouseUp(MouseButtons button) {
			if (button == MouseButtons.Left) {
				if (this.OnLeftUp != null)
					this.OnLeftUp(this, new MouseClickEventArgs(MouseButtons.Left, false, false, false));
			}
			else if (button == MouseButtons.Right) {
				if (this.OnRightUp != null)
					this.OnRightUp(this, new MouseClickEventArgs(MouseButtons.Right, false, false, false));
			}

			if (this.Debug.MouseClick)
				UnityEngine.Debug.Log(String.Format("MouseClick: up {0} at {1}", button, this.mousePosition));
		}

		private void OnMouseClick(MouseButtons button) {
			var buttonIndex = (int) button;

			if (button == MouseButtons.Left) {
				if (this.OnLeftClick != null)
					this.OnLeftClick(this, new MouseClickEventArgs(MouseButtons.Left, true, true, false));
			}
			else if (button == MouseButtons.Right) {
				if (this.OnRightClick != null)
					this.OnRightClick(this, new MouseClickEventArgs(MouseButtons.Right, true, true, false));
			}

			if (this.Debug.MouseClick)
				UnityEngine.Debug.Log(String.Format("MouseClick: click {0} at {1}", button, this.mousePosition));

			// Check if this is first or second click for double click
			if (this.mouseClickTimerEnabled[buttonIndex]) {
				this.OnMouseDoubleClick(button);
				this.mouseClickTimerEnabled[buttonIndex] = false;
				this.mouseClickTimer[buttonIndex] = 0;
			}
			else this.mouseClickTimerEnabled[buttonIndex] = true;
		}

		private void OnMouseDoubleClick(MouseButtons button) {
			if (button == MouseButtons.Left) {
				if (this.OnLeftDoubleClick != null)
					this.OnLeftDoubleClick(this, new MouseClickEventArgs(MouseButtons.Left, false, false, true));
			}
			else if (button == MouseButtons.Right) {
				if (this.OnRightDoubleClick != null)
					this.OnRightDoubleClick(this, new MouseClickEventArgs(MouseButtons.Right, false, false, true));
			}

			if (this.Debug.MouseClick)
				UnityEngine.Debug.Log(String.Format("MouseClick: double click {0} at {1}", button, this.mousePosition));
		}

		public enum MouseOnClickTypes {
			OnUp,
			OnDown
		}

		public enum MouseButtons {
			Left = 0,
			Right = 1,
			Middle = 2,
			Unknown
		}

		#endregion

		[Serializable]
		public class DebugOptions {
			public bool MouseMove;
			public bool MouseClick;
			public bool MouseScroll;
			public bool Keyboard;
		}
	}
}
