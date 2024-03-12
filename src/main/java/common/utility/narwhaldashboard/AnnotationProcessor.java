package common.utility.narwhaldashboard;

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

import common.core.subsystems.NAR_PIDSubsystem;

import javax.lang.model.element.Element; // Import the missing Element class
import javax.lang.model.element.ElementKind; // Import the missing ElementKind class
import javax.annotation.processing.Processor; // Import the missing Processor class

@AutoService(Processor.class)
@SupportedAnnotationTypes("common.utility.narwhaldashboard.NARUpdateable")
@SupportedSourceVersion(SourceVersion.RELEASE_11)
public class AnnotationProcessor extends AbstractProcessor{

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        annotations.forEach(annotation ->
            roundEnv.getElementsAnnotatedWith(annotation).forEach(
                element -> processAnnotation((Element) element)));
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'process'");
    }

    public void processAnnotation(Element element) {
        String methodName = element.getSimpleName().toString();
        
        // String packageName = element.getEnclosingElement().toString();
        // String annotationName = methodName + "AnnotationUpdateable"; // fix later
        // String annotationFullName = packageName + "." + annotationName;

        element.getEnclosedElements()
            .stream().filter(e -> ElementKind.METHOD.equals(e.getKind())).forEach(
                method -> {
                    NARUpdateable annotation = method.getAnnotation(NARUpdateable.class);
                    NarwhalDashboard.getInstance().addUpdate(annotation.name(), ()->{
                        try {
                            Method meth = method.getClass().getDeclaredMethod(methodName);
                            Object o = method.getClass().getDeclaredMethod("getInstance").invoke(null);

                            return meth.invoke(o);
                        } catch (Exception e) {
                            e.printStackTrace();
                            return null;
                        }
                    });
                }

            );

    }
    
}