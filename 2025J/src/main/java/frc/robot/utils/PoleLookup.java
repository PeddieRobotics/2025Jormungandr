package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;

public class PoleLookup {
    // MILSTEIN
    // private static Map<Integer, PoleCategory> kPoleOffsetTable = new HashMap<>() {{
    //     // RED POLES
    //     put(PoleLookup.getPoleNumber(6, AlignmentDestination.LEFT),   PoleCategory.HIGH);
    //     put(PoleLookup.getPoleNumber(6, AlignmentDestination.RIGHT),  PoleCategory.INSANELY_HIGH);

    //     put(PoleLookup.getPoleNumber(7, AlignmentDestination.LEFT),   PoleCategory.HIGH);
    //     put(PoleLookup.getPoleNumber(7, AlignmentDestination.RIGHT),  PoleCategory.EXTREMELY_HIGH);

    //     put(PoleLookup.getPoleNumber(8, AlignmentDestination.LEFT),   PoleCategory.SLIGHTLY_HIGH);
    //     put(PoleLookup.getPoleNumber(8, AlignmentDestination.RIGHT),  PoleCategory.EXTREMELY_HIGH);

    //     put(PoleLookup.getPoleNumber(9, AlignmentDestination.LEFT),   PoleCategory.SLIGHTLY_HIGH);
    //     put(PoleLookup.getPoleNumber(9, AlignmentDestination.RIGHT),  PoleCategory.SLIGHTLY_HIGH);

    //     put(PoleLookup.getPoleNumber(10, AlignmentDestination.LEFT),  PoleCategory.SLIGHTLY_LOW);
    //     put(PoleLookup.getPoleNumber(10, AlignmentDestination.RIGHT), PoleCategory.HIGH);

    //     put(PoleLookup.getPoleNumber(11, AlignmentDestination.LEFT),  PoleCategory.SLIGHTLY_HIGH);
    //     put(PoleLookup.getPoleNumber(11, AlignmentDestination.RIGHT), PoleCategory.EXTREMELY_HIGH);
        

    //     // BLUE POLES
    //     put(PoleLookup.getPoleNumber(17, AlignmentDestination.LEFT),  PoleCategory.SLIGHTLY_HIGH);
    //     put(PoleLookup.getPoleNumber(17, AlignmentDestination.RIGHT), PoleCategory.HIGH);

    //     put(PoleLookup.getPoleNumber(18, AlignmentDestination.LEFT),  PoleCategory.HIGH);
    //     put(PoleLookup.getPoleNumber(18, AlignmentDestination.RIGHT), PoleCategory.HIGH);

    //     put(PoleLookup.getPoleNumber(19, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
    //     put(PoleLookup.getPoleNumber(19, AlignmentDestination.RIGHT), PoleCategory.EXTREMELY_HIGH);

    //     put(PoleLookup.getPoleNumber(20, AlignmentDestination.LEFT),  PoleCategory.SLIGHTLY_HIGH);
    //     put(PoleLookup.getPoleNumber(20, AlignmentDestination.RIGHT), PoleCategory.HIGH);

    //     put(PoleLookup.getPoleNumber(21, AlignmentDestination.LEFT),  PoleCategory.HIGH);
    //     put(PoleLookup.getPoleNumber(21, AlignmentDestination.RIGHT), PoleCategory.HIGH);

    //     put(PoleLookup.getPoleNumber(22, AlignmentDestination.LEFT),  PoleCategory.HIGH);
    //     put(PoleLookup.getPoleNumber(22, AlignmentDestination.RIGHT), PoleCategory.HIGH);
    // }};
    
    // TUNE THESE NUMBERS
    private static Map<Integer, PoleCategory> kPoleOffsetTableMass = new HashMap<>() {{
        // RED POLES
        put(PoleLookup.getPoleNumber(6, AlignmentDestination.LEFT),   PoleCategory.MEDIUM);
        put(PoleLookup.getPoleNumber(6, AlignmentDestination.RIGHT),  PoleCategory.MEDIUM);

        put(PoleLookup.getPoleNumber(7, AlignmentDestination.LEFT),   PoleCategory.HIGH);
        put(PoleLookup.getPoleNumber(7, AlignmentDestination.RIGHT),  PoleCategory.HIGH);

        put(PoleLookup.getPoleNumber(8, AlignmentDestination.LEFT),   PoleCategory.INSANELY_HIGH);
        put(PoleLookup.getPoleNumber(8, AlignmentDestination.RIGHT),  PoleCategory.INSANELY_HIGH);

        put(PoleLookup.getPoleNumber(9, AlignmentDestination.LEFT),   PoleCategory.HIGH);
        put(PoleLookup.getPoleNumber(9, AlignmentDestination.RIGHT),  PoleCategory.EXTREMELY_HIGH);

        put(PoleLookup.getPoleNumber(10, AlignmentDestination.LEFT),  PoleCategory.SLIGHTLY_HIGH);
        put(PoleLookup.getPoleNumber(10, AlignmentDestination.RIGHT), PoleCategory.MEDIUM);

        put(PoleLookup.getPoleNumber(11, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(11, AlignmentDestination.RIGHT), PoleCategory.HIGH);
        

        // BLUE POLES
        put(PoleLookup.getPoleNumber(17, AlignmentDestination.LEFT),  PoleCategory.HIGH);
        put(PoleLookup.getPoleNumber(17, AlignmentDestination.RIGHT), PoleCategory.EXTREMELY_HIGH);

        put(PoleLookup.getPoleNumber(18, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(18, AlignmentDestination.RIGHT), PoleCategory.HIGH);

        put(PoleLookup.getPoleNumber(19, AlignmentDestination.LEFT),  PoleCategory.HIGH);
        put(PoleLookup.getPoleNumber(19, AlignmentDestination.RIGHT), PoleCategory.HIGH);

        put(PoleLookup.getPoleNumber(20, AlignmentDestination.LEFT),  PoleCategory.HIGH);
        put(PoleLookup.getPoleNumber(20, AlignmentDestination.RIGHT), PoleCategory.HIGH);

        put(PoleLookup.getPoleNumber(21, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(21, AlignmentDestination.RIGHT), PoleCategory.MEDIUM);

        put(PoleLookup.getPoleNumber(22, AlignmentDestination.LEFT),  PoleCategory.HIGH);
        put(PoleLookup.getPoleNumber(22, AlignmentDestination.RIGHT), PoleCategory.HIGH);
    }};

    private static Map<Integer, PoleCategory> kPoleOffsetTable = new HashMap<>() {{
        // RED POLES
        put(PoleLookup.getPoleNumber(6, AlignmentDestination.LEFT),   PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(6, AlignmentDestination.RIGHT),  PoleCategory.EXTREMELY_HIGH);

        put(PoleLookup.getPoleNumber(7, AlignmentDestination.LEFT),   PoleCategory.INSANELY_HIGH);
        put(PoleLookup.getPoleNumber(7, AlignmentDestination.RIGHT),  PoleCategory.INSANELY_HIGH);

        put(PoleLookup.getPoleNumber(8, AlignmentDestination.LEFT),   PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(8, AlignmentDestination.RIGHT),  PoleCategory.HIGH);

        put(PoleLookup.getPoleNumber(9, AlignmentDestination.LEFT),   PoleCategory.INSANELY_HIGH);
        put(PoleLookup.getPoleNumber(9, AlignmentDestination.RIGHT),  PoleCategory.EXTREMELY_HIGH);

        put(PoleLookup.getPoleNumber(10, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(10, AlignmentDestination.RIGHT), PoleCategory.HIGH);

        put(PoleLookup.getPoleNumber(11, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(11, AlignmentDestination.RIGHT), PoleCategory.HIGH);
        

        // BLUE POLES
        put(PoleLookup.getPoleNumber(17, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(17, AlignmentDestination.RIGHT), PoleCategory.HIGH);

        put(PoleLookup.getPoleNumber(18, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(18, AlignmentDestination.RIGHT), PoleCategory.INSANELY_HIGH);

        put(PoleLookup.getPoleNumber(19, AlignmentDestination.LEFT),  PoleCategory.EXTREMELY_HIGH);
        put(PoleLookup.getPoleNumber(19, AlignmentDestination.RIGHT), PoleCategory.EXTREMELY_HIGH);

        put(PoleLookup.getPoleNumber(20, AlignmentDestination.LEFT),  PoleCategory.HIGH);
        put(PoleLookup.getPoleNumber(20, AlignmentDestination.RIGHT), PoleCategory.EXTREMELY_HIGH);

        put(PoleLookup.getPoleNumber(21, AlignmentDestination.LEFT),  PoleCategory.MEDIUM);
        put(PoleLookup.getPoleNumber(21, AlignmentDestination.RIGHT), PoleCategory.SLIGHTLY_HIGH);

        put(PoleLookup.getPoleNumber(22, AlignmentDestination.LEFT),  PoleCategory.INSANELY_HIGH);
        put(PoleLookup.getPoleNumber(22, AlignmentDestination.RIGHT), PoleCategory.EXTREMELY_HIGH);
    }};

    private static double getCategoryHeightOffset(PoleCategory category) {
        // approximately 0.1 = 0.5 inches
        switch (category) {
            case EXTREMELY_LOW:
                return -0.15;
            case LOW:
                return -0.1;
            case SLIGHTLY_LOW:
                return -0.05;
            case MEDIUM:
                return 0;
            case SLIGHTLY_HIGH:
                return 0.05;
            case HIGH:
                return 0.1;
            case EXTREMELY_HIGH:
                return 0.15;
            case INSANELY_HIGH:
                return 0.25;
        }
        return 0;
    }

    private enum PoleCategory {
        EXTREMELY_LOW,
        LOW,
        SLIGHTLY_LOW,
        MEDIUM,
        SLIGHTLY_HIGH,
        HIGH,
        EXTREMELY_HIGH,
        INSANELY_HIGH
    };


    private static int getPoleNumber(int tag, AlignmentDestination side) {
        int lowBit = side == AlignmentDestination.LEFT ? 0 : 1;
        return (tag << 1) | lowBit;
    }
    
    public static double lookupPole(int tag, AlignmentDestination side) {
        // PoleCategory category = kPoleOffsetTableMass.get(getPoleNumber(tag, side));
        PoleCategory category = kPoleOffsetTable.get(getPoleNumber(tag, side));
        return getCategoryHeightOffset(category);
    }
}
