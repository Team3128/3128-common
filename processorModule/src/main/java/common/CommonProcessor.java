// package common.utility.narwhaldashboard;
package common;

import java.lang.reflect.Method;
import java.util.List;
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
        annotations.forEach(annotation ->
            roundEnv.getElementsAnnotatedWith(annotation).forEach(
                element -> generateFile((Element) element)));
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'process'");
    }

    private void generateFile(Element element) {
        String className = element.getSimpleName().toString() + "Generated";
        try {
            JavaFileObject file = processingEnv.getFiler().createSourceFile(className);
            try (Writer writer = file.openWriter()) {
                writer.write("public class " + className + " {\n" +
                        "    // This is a generated file for element: " + element.getSimpleName() + "\n" +
                        "}\n");
            }
        } catch (IOException e) {
            processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, "An error occurred: " + e.getMessage());
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