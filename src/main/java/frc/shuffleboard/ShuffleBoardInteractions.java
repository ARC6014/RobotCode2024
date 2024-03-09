package frc.shuffleboard;

import java.util.ArrayList;

import frc.shuffleboard.tabs.OperatorTab;
import frc.shuffleboard.tabs.SwerveTab;

public class ShuffleBoardInteractions {

    // Trims unneccesary tabs when in competition
    public final boolean mDebug = true;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance;

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    // public FieldView mFieldView = new FieldView();
    // private SwerveTab mSwerveTab = new SwerveTab();
    private OperatorTab mOperatorTab = new OperatorTab();

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        // mTabs.add(mSwerveTab);
        mTabs.add(mOperatorTab);
        for (ShuffleboardTabBase tab : mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
        // mFieldView.update();
    }
}
