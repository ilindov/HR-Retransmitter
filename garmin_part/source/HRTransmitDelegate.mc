using Toybox.WatchUi;

class GarminHRBTBroadcastDelegate extends WatchUi.BehaviorDelegate {

    function initialize() {
        BehaviorDelegate.initialize();
    }

    function onMenu() {
        WatchUi.pushView(new Rez.Menus.MainMenu(), new GarminHRBTBroadcastMenuDelegate(), WatchUi.SLIDE_UP);
        return true;
    }

}