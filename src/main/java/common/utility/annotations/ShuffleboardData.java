package common.utility.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface ShuffleboardData {
    public String tab() default "General";
    public String name() default "unspecified";
    public int x() default 0;
    public int y() default 0;
}

