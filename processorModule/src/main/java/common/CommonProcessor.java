// package common.utility.narwhaldashboard;
package common;

// import src.main.java.common.utility.narwhaldashboard.NarwhalDashboard;

import java.lang.reflect.Method;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.TypeElement;

import com.google.auto.service.AutoService;

import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Writer;
import java.util.Set;

// import common.core.subsystems.NAR_PIDSubsystem;

import javax.lang.model.element.Element; // Import the missing Element class
import javax.lang.model.element.ElementKind; // Import the missing ElementKind class
import javax.annotation.processing.Processor; // Import the missing Processor class
import javax.tools.Diagnostic; // Import the missing Diagnostic class

@AutoService(Processor.class)
@SupportedAnnotationTypes("common.utility.narwhaldashboard.NARUpdateable")
@SupportedSourceVersion(SourceVersion.RELEASE_11)
public class CommonProcessor extends AbstractProcessor{

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        processingEnv.getMessager().printMessage(Diagnostic.Kind.WARNING, "Annotation Processor Running...");
        try{
            annotations.forEach(annotation ->
            roundEnv.getElementsAnnotatedWith(annotation).forEach(
                element -> processingEnv.getMessager().printMessage(Diagnostic.Kind.NOTE, element.getSimpleName().toString() + " has been processed")));
        // TODO Auto-generated method stub
        }catch(Exception e){
            e.printStackTrace();
            return false;
        }
        return true;
    }

    private void writeBuilderFile(String className, Map<String, String> setterMap) throws IOException {
        String packageName = null;
        int lastDot = className.lastIndexOf('.');
        if (lastDot > 0) {
            packageName = className.substring(0, lastDot);
        }

        String simpleClassName = className.substring(lastDot + 1);
        String builderClassName = className + "Builder";
        String builderSimpleClassName = builderClassName
        .substring(lastDot + 1);

        JavaFileObject builderFile = processingEnv.getFiler()
        .createSourceFile(builderClassName);
        
        try (PrintWriter out = new PrintWriter(builderFile.openWriter())) {

            if (packageName != null) {
                out.print("package ");
                out.print(packageName);
                out.println(";");
                out.println();
            }

            out.print("public class ");
            out.print(builderSimpleClassName);
            out.println(" {");
            out.println();

            out.print("    private ");
            out.print(simpleClassName);
            out.print(" object = new ");
            out.print(simpleClassName);
            out.println("();");
            out.println();

            out.print("    public ");
            out.print(simpleClassName);
            out.println(" build() {");
            out.println("        return object;");
            out.println("    }");
            out.println();

            setterMap.entrySet().forEach(setter -> {
                String methodName = setter.getKey();
                String argumentType = setter.getValue();

                out.print("    public ");
                out.print(builderSimpleClassName);
                out.print(" ");
                out.print(methodName);

                out.print("(");

                out.print(argumentType);
                out.println(" value) {");
                out.print("        object.");
                out.print(methodName);
                out.println("(value);");
                out.println("        return this;");
                out.println("    }");
                out.println();
            });

            out.println("}");
        }
    }

    public void processAnnotation(Element element) {
        System.out.println("Element: " + element);
        // String methodName = element.getSimpleName().toString();
        
        // // String packageName = element.getEnclosingElement().toString();
        // // String annotationName = methodName + "AnnotationUpdateable"; // fix later
        // // String annotationFullName = packageName + "." + annotationName;
        // processingEnv.getMessager().printMessage(Diagnostic.Kind.WARNING, "Annotation Processor Running For" + methodName);

        // element.getEnclosedElements()
        //     .stream().filter(e -> ElementKind.METHOD.equals(e.getKind())).forEach(
        //         method -> {
        //             NARUpdateable annotation = method.getAnnotation(NARUpdateable.class);
        //             NarwhalDashboard.getInstance().addUpdate(annotation.name(), ()->{
        //                 try {
        //                     Method meth = method.getClass().getDeclaredMethod(methodName);
        //                     Object o = method.getClass().getDeclaredMethod("getInstance").invoke(null);

        //                     return meth.invoke(o);
        //                 } catch (Exception e) {
        //                     e.printStackTrace();
        //                     return null;
        //                 }
        //             });
        //         }

        //     );

    }
    
}