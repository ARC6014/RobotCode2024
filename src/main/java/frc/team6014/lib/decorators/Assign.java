package frc.team6014.lib.decorators;

public @interface Assign {

    String message() default "";

    Prog user() default Prog.Carabelli;

    Prog[] users() default { Prog.Carabelli, Prog.Alia, Prog.CAN, Prog.Xerem, Prog.Ouz };

    public static enum Prog {
        Carabelli,
        Alia,
        CAN,
        Xerem,
        Ouz
    }

}
