package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.CollectionTable;
import jakarta.persistence.Column;
import jakarta.persistence.ElementCollection;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.Table;

import java.util.LinkedHashSet;
import java.util.Set;

@Entity
@Table(name = "ops_user")
public class UserEntity extends BaseOperationalEntity {
    @Column(name = "username", nullable = false, unique = true)
    private String username;
    @Column(name = "display_name")
    private String displayName;
    @Column(name = "password_hash", nullable = false)
    private String passwordHash;
    @Column(name = "active", nullable = false)
    private boolean active = true;
    @ElementCollection(fetch = FetchType.EAGER)
    @CollectionTable(name = "ops_user_role", joinColumns = @JoinColumn(name = "user_id"))
    @Column(name = "role_name")
    private Set<String> roles = new LinkedHashSet<>();

    public String getUsername() { return username; }
    public void setUsername(String username) { this.username = username; }
    public String getDisplayName() { return displayName; }
    public void setDisplayName(String displayName) { this.displayName = displayName; }
    public String getPasswordHash() { return passwordHash; }
    public void setPasswordHash(String passwordHash) { this.passwordHash = passwordHash; }
    public boolean isActive() { return active; }
    public void setActive(boolean active) { this.active = active; }
    public Set<String> getRoles() { return roles; }
    public void setRoles(Set<String> roles) { this.roles = roles; }
}
