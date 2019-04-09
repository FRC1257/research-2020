package frc.robot.commands.paths;

import easypath.FollowPath;
import easypath.Path;

public class RightMiddleHatch extends FollowPath {
    public RightMiddleHatch() {
        super(new Path(t ->
                /* {"start":{"x":41,"y":208},"mid1":{"x":216,"y":205},"mid2":{"x":283,"y":285},"end":{"x":282,"y":191}} */
                (-771 * Math.pow(t, 2) + 498 * t + -9) / (120 * Math.pow(t, 2) + -648 * t + 525),
                277.199)
                , x -> {
                    if (x < 0.15) return 0.6;
                    else if (x < 0.75) return 0.8;
                    else return 0.25;
        });
    }
}
