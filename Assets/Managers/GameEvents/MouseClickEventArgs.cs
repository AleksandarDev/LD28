using System;

namespace Assets.Managers.GameEvents {
	public class MouseClickEventArgs : EventArgs {
		public GameEventsManager.MouseButtons Button;
		public bool IsDown;
		public bool IsClick;
		public bool IsDoubleClick;

		public MouseClickEventArgs(GameEventsManager.MouseButtons button, bool isDown, bool isClick, bool isDoubleClick) {
			this.Button = button;
			this.IsDown = isDown;
			this.IsClick = isClick;
			this.IsDoubleClick = isDoubleClick;
		}
	}
}