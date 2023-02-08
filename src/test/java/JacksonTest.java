import org.junit.Test;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class JacksonTest {

    public class Foo {
        public int i = 0;

    }

    @Test
    public void test00() throws JsonProcessingException {
        Foo foo = new Foo();
        ObjectMapper objectMapper = new ObjectMapper();
        System.out.println(objectMapper.writeValueAsString(foo));
    }

}
