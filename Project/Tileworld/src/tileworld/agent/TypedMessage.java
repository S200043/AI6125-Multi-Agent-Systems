package tileworld.agent;

public class TypedMessage extends Message {
	private Object[] object; // the communicated object

	public TypedMessage(String from, String to, String type, Object[] object) {
		super(from, to, type);
		this.object = object;
	}

	public Object[] getObject() {
		return object;
	}

}
