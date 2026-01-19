package frc.robot.controller;

/**
 * Manages switching between different controllers.
 */
public class ControllerManager {

    private Controller activeController;
    private final Controller defaultController;

    public ControllerManager(Controller defaultController) {
        this.defaultController = defaultController;
        this.activeController = defaultController;
    }

    public Controller getActiveController() {
        return activeController;
    }

    public void setActiveController(Controller controller) {
        if (activeController != controller) {
            activeController.onDeactivate();
            activeController = controller;
            activeController.onActivate();
        }
    }

    public void resetToDefault() {
        setActiveController(defaultController);
    }

    public String getActiveControllerName() {
        return activeController.getName();
    }
}
